"""
simulator.py  — Датасет-тэй бүрэн холбогдсон хувилбар
══════════════════════════════════════════════════════════
Өөрчлөлтүүд:
  1. apply_dataset_config()  — датасетаас тохиргоо хүлээн авна
  2. _create_initial_queues() — датасетийн утгуудыг ашиглана
  3. spawn_chance / arrival_rate / discharge_rate — динамик
  4. weather_factor — цаг агаарын нөлөөг хурдад тусгана
  5. MAX_ACTIVE_VEHICLES — датасетаас авна
  6. Анхны машинуудыг датасетаас spawn хийнэ
"""
from __future__ import annotations

import asyncio
import math
import random
from contextlib import suppress
from typing import Any

from fastapi import WebSocket
from starlette.websockets import WebSocketDisconnect

from backend.models.intersection import (
    ComparisonStats,
    QueueHistoryResponse,
    SimulationStartRequest,
    SimulationState,
)
from backend.services.ai_controller import (
    DIRECTIONS,
    LANE_X,
    LANE_Y,
    calculate_green_time,
    get_safe_speed,
    should_yield,
)

CANVAS_WIDTH  = 500.0
CANVAS_HEIGHT = 400.0
SPAWN_OFFSET  = 40.0
OFF_SCREEN_NORTH = -SPAWN_OFFSET
OFF_SCREEN_SOUTH = CANVAS_HEIGHT + SPAWN_OFFSET
OFF_SCREEN_EAST  = -SPAWN_OFFSET
OFF_SCREEN_WEST  = CANVAS_WIDTH  + SPAWN_OFFSET
EXIT_MARGIN = 80.0

BASE_VEHICLE_SPEED  = 3.15
BASE_SPAWN_CHANCE   = 0.45
PEAK_SPAWN_CHANCE   = 0.78
MAX_ACTIVE_VEHICLES = 56
CENTER_X = CANVAS_WIDTH  / 2
CENTER_Y = CANVAS_HEIGHT / 2
PHYSICS_HZ           = 12.0
PHYSICS_STEP_SECONDS = 1.0 / PHYSICS_HZ

DIRECTION_ANGLE = {
    "north": math.pi,
    "east":  math.pi / 2,
    "south": 0.0,
    "west": -math.pi / 2,
}

DIRECTION_VECTOR = {
    "north": (0.0,  1.0),
    "south": (0.0, -1.0),
    "east":  (1.0,  0.0),
    "west":  (-1.0, 0.0),
}

VEHICLE_DYNAMICS = {
    "car":       {"cruise": 42.0, "accel": 58.0, "brake": 92.0},
    "truck":     {"cruise": 32.0, "accel": 38.0, "brake": 70.0},
    "bus":       {"cruise": 30.0, "accel": 34.0, "brake": 64.0},
    "emergency": {"cruise": 48.0, "accel": 70.0, "brake": 100.0},
}

INTERSECTION_POSITIONS = [
    {"id": "A", "row": 0, "col": 0, "label": "A"},
    {"id": "B", "row": 0, "col": 1, "label": "B"},
    {"id": "C", "row": 0, "col": 2, "label": "C"},
    {"id": "D", "row": 1, "col": 0, "label": "D"},
    {"id": "E", "row": 1, "col": 1, "label": "E"},
    {"id": "F", "row": 1, "col": 2, "label": "F"},
    {"id": "G", "row": 2, "col": 0, "label": "G"},
    {"id": "H", "row": 2, "col": 1, "label": "H"},
    {"id": "I", "row": 2, "col": 2, "label": "I"},
]

VEHICLE_COLORS: dict[str, str] = {
    "north": "#ff6d00",
    "south": "#00e5ff",
    "east":  "#ffd600",
    "west":  "#c653ff",
}


class TrafficSimulator:
    def __init__(self) -> None:
        self._lock        = asyncio.Lock()
        self._subscribers: set[WebSocket] = set()
        self._loop_task: asyncio.Task[None] | None = None
        self._rng = random.Random()
   

        self.mode      = "fixed"
        self.peak_hour = False
        self.heavy_north = False
        self.bus_directions:       list[str] = []
        self.emergency_directions: list[str] = []

        self.is_running   = False
        self.speed        = 1.0

        self.active_dir   = "north"
        self.dir_index    = 0
        self.signal_state = "green"
        self.phase_timer  = 30

        # ── Датасетаас авах динамик параметрүүд ──────────────────
        self._spawn_chance:   float = BASE_SPAWN_CHANCE
        self._arrival_rate:   float = 0.30
        self._discharge_rate: int   = 2
        self._weather_factor: float = 1.0    # 1.0 = Clear, 0.4 = Ice
        self._max_vehicles:   int   = MAX_ACTIVE_VEHICLES
        self._dataset_loaded: bool  = False   # датасет ачаалагдсан эсэх
        self._dataset_meta:   dict  = {}      # уулзвар, нэр, дүүрэг гэх мэт

        self.queues      = self._create_initial_queues(False, False)
        self.lane_queues = self._create_initial_lane_queues(self.queues)
        self.total_passed = 0
        self.wait_times: dict[str, list[int]] = {"fixed": [], "ai": []}
        self.vehicles: list[dict[str, Any]]   = []
        self.intersections = self._create_intersections()

        self.sim_time = 0
        self.history: list[dict[str, int]] = []
        self.green_times = {direction: 30 for direction in DIRECTIONS}

        self._frame              = 0
        self._vehicle_id         = 0
        self._second_accumulator = 0.0

    # ═══════════════════════════════════════════════════════════
    # ДАТАСЕТ ХОЛБОЛТ — ШИНЭ МЕТОД
    # ═══════════════════════════════════════════════════════════

    async def apply_dataset_config(self, config: dict[str, Any]) -> None:
        """
        DatasetBridge.build_simulator_config()-аас ирсэн dict-г
        симулятор руу шилжүүлнэ.

        Дуудах жишээ (main.py lifespan дотор):
            bridge  = DatasetBridge(loader)
            cfg     = bridge.build_simulator_config(1, use_peak=True, mode="fixed")
            await simulator.apply_dataset_config(cfg)
        """
        async with self._lock:
            # ── Горим ─────────────────────────────────────────
            if "mode" in config:
                self.mode = config["mode"]
            if "peak_hour" in config:
                self.peak_hour = config["peak_hour"]
            if "heavy_north" in config:
                self.heavy_north = config["heavy_north"]
            if "bus_directions" in config:
                self.bus_directions = list(config["bus_directions"])
            if "emergency_directions" in config:
                self.emergency_directions = list(config["emergency_directions"])

            # ── Дараалал ─────────────────────────────────────
            if "queues" in config:
                self.queues = dict(config["queues"])
            if "lane_queues" in config:
                self.lane_queues = dict(config["lane_queues"])

            # ── Гэрлэн дохионы хугацаа ───────────────────────
            if "green_times" in config:
                self.green_times = dict(config["green_times"])
                pair = self._phase_directions()
                self.phase_timer = max(
                    self.green_times.get(pair[0], 30),
                    self.green_times.get(pair[1], 30),
                )
            if "phase_timer" in config:
                self.phase_timer = int(config["phase_timer"])

            # ── Датасетаас ирсэн машинуудыг нэмнэ ──────────
            if "vehicles" in config and config["vehicles"]:
                # Одоо байгаа машинуудыг цэвэрлэж датасетийнхийг тавина
                self.vehicles = list(config["vehicles"])
                # vehicle_id-г давтахгүйн тулд max-г олно
                if self.vehicles:
                    self._vehicle_id = max(v.get("id", 0) for v in self.vehicles)

            # ── Динамик параметрүүд ───────────────────────────
            if "spawn_chance" in config:
                self._spawn_chance = float(config["spawn_chance"])
            if "arrival_rate" in config:
                self._arrival_rate = float(config["arrival_rate"])
            if "discharge_rate" in config:
                self._discharge_rate = int(config["discharge_rate"])
            if "weather_factor" in config:
                self._weather_factor = float(config["weather_factor"])
            if "max_vehicles" in config:
                self._max_vehicles = int(config["max_vehicles"])

            # ── Мета мэдээлэл хадгалах ────────────────────────
            self._dataset_loaded = True
            self._dataset_meta = {
                k: config.get(k, "")
                for k in ["intersection_id", "intersection_name",
                           "district", "timestamp", "congestion_index",
                           "congestion_tier", "weather", "load_factors",
                           "kpi_baseline"]
            }

            self._refresh_green_times()

        print(
            f"[simulator] Датасет ачааллаа: "
            f"{config.get('intersection_name', '?')} | "
            f"дараалал={sum(self.queues.values())} | "
            f"машин={len(self.vehicles)} | "
            f"цаг агаар={config.get('weather', '?')} | "
            f"оргил={self.peak_hour}"
        )

    # ═══════════════════════════════════════════════════════════
    # ЦИКЛ
    # ═══════════════════════════════════════════════════════════

    async def start_loop(self) -> None:
        if self._loop_task is not None and not self._loop_task.done():
            return
        self._loop_task = asyncio.create_task(self._run_loop())

    async def stop_loop(self) -> None:
        if self._loop_task is None:
            return
        self._loop_task.cancel()
        with suppress(asyncio.CancelledError):
            await self._loop_task
        self._loop_task = None

    async def _run_loop(self) -> None:
        while True:
            async with self._lock:
                running = self.is_running
                speed   = self.speed
            if not running:
                await asyncio.sleep(0.1)
                continue
            step_seconds = PHYSICS_STEP_SECONDS
            await asyncio.sleep(max(0.01, step_seconds / max(speed, 0.1)))
            state = await self._tick(step_seconds)
            await self._broadcast_state(state)

    # ═══════════════════════════════════════════════════════════
    # ДАРААЛАЛ ЭХЛҮҮЛЭХ
    # ═══════════════════════════════════════════════════════════

    def _create_initial_lane_queues(
        self, direction_queues: dict[str, int]
    ) -> dict[str, int]:
        lane_queues: dict[str, int] = {}
        for direction in DIRECTIONS:
            total = max(0, direction_queues.get(direction, 0))
            lane_queues[f"{direction}_0"] = total // 2
            lane_queues[f"{direction}_1"] = total - (total // 2)
        return lane_queues

    def _sum_direction_queues(
        self, lane_queues: dict[str, int]
    ) -> dict[str, int]:
        return {
            direction: (
                lane_queues.get(f"{direction}_0", 0) +
                lane_queues.get(f"{direction}_1", 0)
            )
            for direction in DIRECTIONS
        }

    def _phase_directions(
        self, active_dir: str | None = None
    ) -> tuple[str, str]:
        current = active_dir or self.active_dir
        if current in {"north", "south"}:
            return ("north", "south")
        return ("east", "west")

    def _create_initial_queues(
        self, peak_hour: bool, heavy_north: bool
    ) -> dict[str, int]:
        """
        Датасет ачаалагдсан бол self.queues-г шууд буцаана.
        Ачаалагдаагүй бол default утгуудыг ашиглана.
        """
        return {
            "north": 26 if peak_hour and heavy_north else 17 if peak_hour else 20 if heavy_north else 9,
            "south": 14 if peak_hour else 7,
            "east":  12 if peak_hour else 6,
            "west":  12 if peak_hour else 7,
        }

    def _create_intersections(self) -> list[dict[str, Any]]:
        intersections: list[dict[str, Any]] = []
        for pos in INTERSECTION_POSITIONS:
            direction_queues = {
                "north": self._rng.randint(2, 10),
                "south": self._rng.randint(2, 10),
                "east":  self._rng.randint(2, 10),
                "west":  self._rng.randint(2, 10),
            }
            lane_queues = self._create_initial_lane_queues(direction_queues)
            intersections.append(
                {
                    **pos,
                    "queues":      direction_queues,
                    "laneQueues":  lane_queues,
                    "activeDir":   self._rng.choice(("north", "east")),
                    "signalState": "green",
                    "timer":       self._rng.randint(10, 30),
                    "greenTimes":  calculate_green_time(direction_queues),
                }
            )
        return intersections

    def _avg_wait(self, waits: list[int]) -> int:
        if not waits:
            return 0
        return int(round(sum(waits) / len(waits)))

    def _build_vehicle_type_counts(self) -> dict[str, dict[str, int]]:
        type_counts: dict[str, dict[str, int]] = {
            direction: {} for direction in DIRECTIONS
        }
        for vehicle in self.vehicles:
            direction     = vehicle["dir"]
            vehicle_type  = vehicle["type"]
            bucket        = type_counts[direction]
            bucket[vehicle_type] = bucket.get(vehicle_type, 0) + 1
        return type_counts

    def _refresh_green_times(self) -> None:
        if self.mode == "ai":
            self.green_times = calculate_green_time(
                self.queues,
                is_peak_hour=self.peak_hour,
                bus_directions=self.bus_directions,
                emergency_directions=self.emergency_directions,
                vehicle_type_counts=self._build_vehicle_type_counts(),
            )
        else:
            # Тогтмол горимд: датасетийн бодит green_sec ашиглана
            # (apply_dataset_config-аар тохируулагдсан байна)
            if not any(self.green_times.values()):
                self.green_times = {direction: 30 for direction in DIRECTIONS}

    # ═══════════════════════════════════════════════════════════
    # МАШИН ҮҮСГЭХ
    # ═══════════════════════════════════════════════════════════

    def _spawn_vehicle(self, direction: str) -> dict[str, Any]:
        self._vehicle_id += 1
        turn = self._rng.choices(
            ["straight", "left", "right"],
            weights=[0.55, 0.22, 0.23],
            k=1,
        )[0]
        lane_idx = 0 if turn == "left" else (1 if turn == "right" else self._rng.randint(0, 1))

        if direction == "north":
            x = LANE_X["north"][lane_idx]
            y = OFF_SCREEN_NORTH
        elif direction == "south":
            x = LANE_X["south"][lane_idx]
            y = OFF_SCREEN_SOUTH
        elif direction == "east":
            x = OFF_SCREEN_EAST
            y = LANE_Y["east"][lane_idx]
        else:
            x = OFF_SCREEN_WEST
            y = LANE_Y["west"][lane_idx]

        vehicle_type = self._rng.choice(["car", "car", "car", "bus", "truck"])
        if direction in self.bus_directions and self._rng.random() < 0.4:
            vehicle_type = "bus"

        # Цаг агаарын нөлөөгөөр хурд бууруулна
        cruise = VEHICLE_DYNAMICS[vehicle_type]["cruise"] * self._weather_factor

        return {
            "id":           self._vehicle_id,
            "dir":          direction,
            "type":         vehicle_type,
            "lane":         lane_idx,
            "turn":         turn,
            "x":            float(x),
            "y":            float(y),
            "speed":        0.0,
            "targetSpeed":  cruise,
            "angle":        DIRECTION_ANGLE[direction],
            "steer":        0.0,
            "suspension":   self._rng.uniform(-0.15, 0.15),
            "turnProgress": 0.0,
            "turnStartX":   None,
            "turnStartY":   None,
            "turnEndX":     None,
            "turnEndY":     None,
            "turnFromDir":  None,
            "turnToDir":    None,
            "waiting":      False,
            "color":        VEHICLE_COLORS[direction],
            "fromDataset":  False,
            "weatherFactor": self._weather_factor,
        }

    def _is_too_close_to_existing(self, new_vehicle: dict[str, Any]) -> bool:
        min_dist = 52.0 if self.peak_hour else 96.0
        new_x    = float(new_vehicle["x"])
        new_y    = float(new_vehicle["y"])
        for v in self.vehicles:
            if (v["dir"] != new_vehicle["dir"] or
                    v.get("lane") != new_vehicle.get("lane")):
                continue
            dist = (
                abs(float(v["y"]) - new_y)
                if new_vehicle["dir"] in {"north", "south"}
                else abs(float(v["x"]) - new_x)
            )
            if dist < min_dist:
                return True
        return False

    # ═══════════════════════════════════════════════════════════
    # ХӨДӨЛГӨӨН
    # ═══════════════════════════════════════════════════════════

    def _turned_direction(self, current_dir: str, turn: str) -> str:
        if turn == "straight":
            return current_dir
        if current_dir == "north":
            return "east" if turn == "left" else "west"
        if current_dir == "south":
            return "west" if turn == "left" else "east"
        if current_dir == "east":
            return "south" if turn == "left" else "north"
        return "north" if turn == "left" else "south"

    def _can_turn_now(self, vehicle: dict[str, Any]) -> bool:
        d, x, y = vehicle["dir"], float(vehicle["x"]), float(vehicle["y"])
        if d == "north": return y >= CENTER_Y - 6
        if d == "south": return y <= CENTER_Y + 6
        if d == "east":  return x >= CENTER_X - 6
        return x <= CENTER_X + 6

    def _stop_distance(self, direction: str, x: float, y: float) -> float:
        if direction == "north": return 128.0 - y
        if direction == "south": return y - 272.0
        if direction == "east":  return 178.0 - x
        return x - 322.0

    def _angle_lerp(
        self, current: float, target: float, ratio: float
    ) -> float:
        delta = (target - current + math.pi) % (math.pi * 2) - math.pi
        return current + delta * max(0.0, min(1.0, ratio))

    def _turn_endpoint(
        self, next_dir: str, lane: int
    ) -> tuple[float, float]:
        if next_dir in {"north", "south"}:
            return (
                LANE_X[next_dir][lane],
                CENTER_Y + (42.0 if next_dir == "north" else -42.0),
            )
        return (
            CENTER_X + (42.0 if next_dir == "east" else -42.0),
            LANE_Y[next_dir][lane],
        )

    def _advance_turn(
        self, vehicle: dict[str, Any], travel_px: float
    ) -> dict[str, Any]:
        sx  = float(vehicle.get("turnStartX") or vehicle["x"])
        sy  = float(vehicle.get("turnStartY") or vehicle["y"])
        ex  = float(vehicle.get("turnEndX")   or vehicle["x"])
        ey  = float(vehicle.get("turnEndY")   or vehicle["y"])
        fd  = vehicle.get("turnFromDir") or vehicle["dir"]
        td  = vehicle.get("turnToDir")   or vehicle["dir"]
        progress = min(
            1.0,
            float(vehicle.get("turnProgress", 0.0)) + max(0.018, travel_px / 42.0)
        )
        eased = progress * progress * (3.0 - 2.0 * progress)
        cx, cy = CENTER_X, CENTER_Y
        x = (1 - eased)**2 * sx + 2*(1-eased)*eased*cx + eased**2*ex
        y = (1 - eased)**2 * sy + 2*(1-eased)*eased*cy + eased**2*ey
        angle = self._angle_lerp(DIRECTION_ANGLE[fd], DIRECTION_ANGLE[td], eased)
        turn_sign = 1.0 if (DIRECTION_ANGLE[td] - DIRECTION_ANGLE[fd]) > 0 else -1.0

        updated = {
            **vehicle,
            "x":            x,
            "y":            y,
            "angle":        angle,
            "steer":        math.sin(progress * math.pi) * 0.18 * turn_sign,
            "suspension":   math.sin(
                (self._frame + vehicle["id"]) * 0.22
            ) * min(1.0, float(vehicle.get("speed", 0.0)) / 44.0),
            "turnProgress": progress,
            "waiting":      False,
        }
        if progress >= 1.0:
            updated.update({
                "dir":          td,
                "turn":         "straight",
                "angle":        DIRECTION_ANGLE[td],
                "steer":        0.0,
                "turnProgress": 0.0,
                "turnStartX":   None, "turnStartY": None,
                "turnEndX":     None, "turnEndY":   None,
                "turnFromDir":  None, "turnToDir":  None,
            })
        return updated

    def _move_vehicle(
        self,
        vehicle: dict[str, Any],
        all_vehicles: list[dict[str, Any]],
        dt: float,
    ) -> dict[str, Any] | None:
        direction    = vehicle["dir"]
        x            = float(vehicle["x"])
        y            = float(vehicle["y"])
        turn         = vehicle.get("turn", "straight")
        current_speed = float(vehicle.get("speed", 0.0))
        dynamics      = VEHICLE_DYNAMICS.get(vehicle.get("type", "car"), VEHICLE_DYNAMICS["car"])

        # Цаг агаарын хурдны хязгаар
        weather_factor = float(vehicle.get("weatherFactor", self._weather_factor))
        max_cruise     = dynamics["cruise"] * weather_factor

        same_lane = [
            v for v in all_vehicles
            if v is not vehicle and v["dir"] == direction
            and v.get("lane") == vehicle.get("lane")
        ]

        # Аюулгүй хурд тооцох (ai_controller.get_safe_speed ашиглана)
        safe_fraction = get_safe_speed(vehicle, same_lane, base_speed=1.0)
        target_speed  = max_cruise * safe_fraction

        # Зогсоолын шугамд зогсох
        must_yield = False
        if vehicle.get("turnProgress", 0.0) < 0.01:
            stop_dist = self._stop_distance(direction, x, y)
            if stop_dist > 0:
                green_dirs = self._phase_directions()
                is_green = (
                    self.signal_state == "green"
                    and direction in green_dirs
                )

                must_yield = not is_green
                if must_yield:
                    slow_range = 70.0
                    if stop_dist < slow_range:
                        ratio        = stop_dist / slow_range
                        target_speed = min(target_speed, max_cruise * ratio * ratio)
                    if stop_dist < 14.0:
                        target_speed = 0.0

        speed_delta = target_speed - current_speed
        if speed_delta >= 0:
            speed_px = current_speed + min(speed_delta, dynamics["accel"] * dt)
        else:
            speed_px = current_speed + max(speed_delta, -dynamics["brake"] * dt)
        speed_px  = max(0.0, speed_px)
        travel_px = speed_px * dt

        # Эргэлт
        if vehicle.get("turnProgress", 0.0) > 0:
            moved = self._advance_turn({**vehicle, "speed": speed_px}, travel_px)
            mx, my = float(moved["x"]), float(moved["y"])
            if (mx < OFF_SCREEN_EAST - EXIT_MARGIN or mx > OFF_SCREEN_WEST + EXIT_MARGIN or
                    my < OFF_SCREEN_NORTH - EXIT_MARGIN or my > OFF_SCREEN_SOUTH + EXIT_MARGIN):
                return None
            return moved

        if speed_px < 0.04:
            return {
                **vehicle,
                "speed":       0.0,
                "targetSpeed": target_speed,
                "angle":       self._angle_lerp(
                    float(vehicle.get("angle", DIRECTION_ANGLE[direction])),
                    DIRECTION_ANGLE[direction], 0.35,
                ),
                "steer":       0.0,
                "suspension":  0.0,
                "waiting":     must_yield or safe_fraction < 0.05,
            }

        dx, dy = DIRECTION_VECTOR[direction]
        x += dx * travel_px
        y += dy * travel_px

        if turn != "straight" and self._can_turn_now({**vehicle, "x": x, "y": y}):
            next_dir     = self._turned_direction(direction, turn)
            end_x, end_y = self._turn_endpoint(next_dir, vehicle.get("lane", 0))
            return self._advance_turn(
                {
                    **vehicle,
                    "x": x, "y": y,
                    "speed": speed_px, "targetSpeed": target_speed,
                    "turnProgress": 0.01,
                    "turnStartX": x, "turnStartY": y,
                    "turnEndX": end_x, "turnEndY": end_y,
                    "turnFromDir": direction, "turnToDir": next_dir,
                },
                travel_px,
            )

        if (x < OFF_SCREEN_EAST - EXIT_MARGIN or x > OFF_SCREEN_WEST + EXIT_MARGIN or
                y < OFF_SCREEN_NORTH - EXIT_MARGIN or y > OFF_SCREEN_SOUTH + EXIT_MARGIN):
            return None

        return {
            **vehicle,
            "x": x, "y": y,
            "dir": direction, "turn": turn,
            "speed": speed_px, "targetSpeed": target_speed,
            "angle": self._angle_lerp(
                float(vehicle.get("angle", DIRECTION_ANGLE[direction])),
                DIRECTION_ANGLE[direction], 0.3,
            ),
            "steer":      0.0,
            "suspension": math.sin(
                (self._frame + vehicle["id"]) * 0.22
            ) * min(1.0, speed_px / 44.0),
            "waiting":    False,
        }

    # ═══════════════════════════════════════════════════════════
    # TICK — үндсэн симуляцийн цикл
    # ═══════════════════════════════════════════════════════════

    async def _tick(self, dt: float = 1.0) -> SimulationState:
        async with self._lock:
            if not self.is_running:
                return self._snapshot_locked()

            self._frame              += 1
            self._second_accumulator += dt
            elapsed_seconds           = int(self._second_accumulator)
            if elapsed_seconds:
                self._second_accumulator -= elapsed_seconds

            for _ in range(elapsed_seconds):
                self.phase_timer -= 1
                if self.phase_timer <= 0:
                    if self.signal_state == "green":
                        self.signal_state = "yellow"
                        self.phase_timer  = 3
                    elif self.signal_state == "yellow":
                        self.signal_state = "all_red"
                        self.phase_timer  = 2
                    else:
                        if self.emergency_directions:
                            self.active_dir = (
                                "north"
                                if self.emergency_directions[0] in {"north", "south"}
                                else "east"
                            )
                        else:
                            self.active_dir = (
                                "east"
                                if self.active_dir in {"north", "south"}
                                else "north"
                            )
                        self.signal_state = "green"
                        self._refresh_green_times()
                        pair     = self._phase_directions()
                        ai_phase = max(self.green_times[pair[0]], self.green_times[pair[1]])
                        self.phase_timer = (
                            # Тогтмол горимд: датасетийн green_sec ашиглана
                            int(self.green_times.get(pair[0], 30))
                            if self.mode == "fixed"
                            else max(12, min(90, ai_phase))
                        )

                # ── Дараалал шинэчлэх ─────────────────────────────
                updated_queues = dict(self.queues)

                # Датасетаас авсан arrival_rate ашиглана
                base_arrival = self._arrival_rate
                north_arrival = (
                    min(0.99, base_arrival + 0.20)
                    if self.heavy_north
                    else base_arrival
                )

                for direction in DIRECTIONS:
                    arrival = north_arrival if direction == "north" else base_arrival
                    if self._rng.random() < arrival:
                        updated_queues[direction] = min(
                            120 if self._dataset_loaded else 60,
                            updated_queues[direction] + self._rng.randint(1, 2)
                        )
                        lk = f"{direction}_{self._rng.randint(0, 1)}"
                        self.lane_queues[lk] = min(
                            80 if self._dataset_loaded else 40,
                            self.lane_queues.get(lk, 0) + 1
                        )

                # ── Ногоон гэрэлд машин нэвтрүүлэх ──────────────
                if self.signal_state == "green":
                    for current_dir in self._phase_directions():
                        if updated_queues[current_dir] <= 0:
                            continue
                        # Датасетийн discharge_rate ашиглана
                        discharge = self._discharge_rate
                        if self.mode == "ai":
                            discharge += 1   # AI нэг машин нэмж гаргана
                        if current_dir in self.bus_directions:
                            discharge += 1
                        if current_dir in self.emergency_directions:
                            discharge += 2
                        moved = min(discharge, updated_queues[current_dir])
                        updated_queues[current_dir] = max(0, updated_queues[current_dir] - moved)
                        self.total_passed += moved
                        for li in (0, 1):
                            lk = f"{current_dir}_{li}"
                            lane_moved = min(
                                moved // 2 + (1 if li == 0 and moved % 2 else 0),
                                self.lane_queues.get(lk, 0),
                            )
                            self.lane_queues[lk] = max(
                                0, self.lane_queues.get(lk, 0) - lane_moved
                            )

                    active_cycle = (
                        max(self.green_times[d] for d in self._phase_directions())
                        if self.mode == "ai"
                        else self.green_times.get(self._phase_directions()[0], 30)
                    )
                    wait_val   = max(1, active_cycle - self.phase_timer)
                    mode_waits = self.wait_times[self.mode]
                    self.wait_times[self.mode] = [*mode_waits[-39:], wait_val]

                self.queues = updated_queues

                # ── 9 уулзварын бяцхан симуляц ────────────────────
                next_intersections: list[dict[str, Any]] = []
                for intersection in self.intersections:
                    nt  = intersection["timer"] - 1
                    nad = intersection["activeDir"]
                    ns  = intersection.get("signalState", "green")
                    nq  = dict(intersection["queues"])
                    nl  = dict(intersection.get(
                        "laneQueues",
                        self._create_initial_lane_queues(nq)
                    ))

                    for direction in DIRECTIONS:
                        if self._rng.random() < self._arrival_rate * 0.6:
                            nq[direction] = min(24, nq[direction] + 1)
                            lk = f"{direction}_{self._rng.randint(0, 1)}"
                            nl[lk] = min(20, nl.get(lk, 0) + 1)

                    if ns == "green":
                        for direction in self._phase_directions(nad):
                            if nq[direction] > 0:
                                nq[direction] = max(0, nq[direction] - 1)

                    if nt <= 0:
                        if ns == "green":
                            ns, nt = "yellow", 3
                        elif ns == "yellow":
                            ns, nt = "all_red", 2
                        else:
                            nad = "east" if nad in {"north", "south"} else "north"
                            ns  = "green"
                            g_times = calculate_green_time(nq)
                            pair    = self._phase_directions(nad)
                            nt      = (
                                30
                                if self.mode == "fixed"
                                else max(12, min(90, max(g_times[pair[0]], g_times[pair[1]]) + 4))
                            )

                    next_intersections.append({
                        **intersection,
                        "timer":      nt,
                        "activeDir":  nad,
                        "queues":     nq,
                        "laneQueues": nl,
                        "signalState": ns,
                        "greenTimes": calculate_green_time(nq),
                    })
                self.intersections = next_intersections

                self.sim_time += 1
                total_queue   = sum(self.queues.values())
                self.history  = [
                    *self.history[-119:],
                    {"t": self.sim_time, "queue": total_queue}
                ]

            # ── Машин үүсгэх ─────────────────────────────────────
            effective_spawn = (
                self._spawn_chance
                if self._dataset_loaded
                else (PEAK_SPAWN_CHANCE if self.peak_hour else BASE_SPAWN_CHANCE)
            )
            max_v = self._max_vehicles if self._dataset_loaded else MAX_ACTIVE_VEHICLES

            if self._rng.random() < effective_spawn * dt and len(self.vehicles) < max_v:
                primary = (
                    "north"
                    if self.heavy_north and self._rng.random() < 0.55
                    else self._rng.choice(DIRECTIONS)
                )
                v1 = self._spawn_vehicle(primary)
                if not self._is_too_close_to_existing(v1):
                    self.vehicles.append(v1)

                if self.peak_hour and self._rng.random() < 0.4 * dt and len(self.vehicles) < max_v:
                    v2 = self._spawn_vehicle(self._rng.choice(DIRECTIONS))
                    if not self._is_too_close_to_existing(v2):
                        self.vehicles.append(v2)

            # ── Машинуудыг хөдөлгөх ──────────────────────────────
            moved: list[dict[str, Any]] = []
            for vehicle in self.vehicles:
                result = self._move_vehicle(vehicle, self.vehicles, dt)
                if result is not None:
                    moved.append(result)
                else:
                    # Canvas-аас гарахдаа waited машин → нийт_passed нэмэх
                    pass
            self.vehicles = moved

            self._refresh_green_times()
            return self._snapshot_locked()

    # ═══════════════════════════════════════════════════════════
    # SNAPSHOT
    # ═══════════════════════════════════════════════════════════

    def _snapshot_locked(self) -> SimulationState:
        # ── kpis: зөвхөн float утгууд (dict[str, float]) ─────────────────
        # "intersection_name" (str) болон "dataset_loaded" (bool) утгуудыг
        # kpis-д оруулж байсан нь Pydantic ValidationError үүсгэж байв.
        # Эдгээрийг SimulationState-н тусдаа талбарт шилжүүлэв.
        congestion_index = self._dataset_meta.get("congestion_index", 0.0)
        kpis: dict[str, float] = {
            "avg_wait":           float(self._avg_wait(self.wait_times[self.mode])),
            "throughput_per_min": round((self.total_passed / max(1, self.sim_time)) * 60.0, 2),
            "avg_queue_depth":    round(sum(self.queues.values()) / max(1, len(self.queues)), 2),
            "queue_stability":    float(
                max(0, 100 - (max(self.queues.values()) - min(self.queues.values())) * 4)
            ),
            "weather_factor":     float(self._weather_factor),
            # congestion_index нь float байх ёстой; string ирвэл 0.0 болгоно
            "congestion_index":   float(congestion_index) if congestion_index != "" else 0.0,
        }

        return SimulationState(
            mode=self.mode,
            peakHour=self.peak_hour,
            heavyNorth=self.heavy_north,
            isRunning=self.is_running,
            speed=round(self.speed, 2),
            activeDir=self.active_dir,
            phaseTimer=int(math.ceil(self.phase_timer)),
            signalState=self.signal_state,
            queues=dict(self.queues),
            laneQueues=dict(self.lane_queues),
            totalPassed=self.total_passed,
            waitTimes={
                "fixed": list(self.wait_times["fixed"]),
                "ai":    list(self.wait_times["ai"]),
            },
            vehicles=list(self.vehicles),
            intersections=list(self.intersections),
            simTime=self.sim_time,
            history=list(self.history),
            greenTimes=dict(self.green_times),
            avgFixedWait=self._avg_wait(self.wait_times["fixed"]),
            avgAIWait=self._avg_wait(self.wait_times["ai"]),
            busDirections=list(self.bus_directions),
            emergencyDirections=list(self.emergency_directions),
            kpis=kpis,
            # ── string/bool мета утгууд — тусдаа талбарт ─────────────────
            intersectionName=str(self._dataset_meta.get("intersection_name", "")),
            datasetLoaded=bool(self._dataset_loaded),
        )

    # ═══════════════════════════════════════════════════════════
    # PUBLIC API
    # ═══════════════════════════════════════════════════════════

    async def get_state(self) -> SimulationState:
        async with self._lock:
            return self._snapshot_locked()

    async def start(self, payload: SimulationStartRequest) -> SimulationState:
        async with self._lock:
            if payload.mode is not None:
                self.mode = payload.mode
            if payload.peak_hour is not None:
                self.peak_hour = payload.peak_hour
            if payload.heavy_north is not None:
                self.heavy_north = payload.heavy_north
            if payload.bus_directions is not None:
                self.bus_directions = list(payload.bus_directions)
            if payload.emergency_directions is not None:
                self.emergency_directions = list(payload.emergency_directions)
            if payload.reset:
                self._reset_runtime_state()
            if payload.autostart:
                self.is_running = True
            self._refresh_green_times()
            if self.mode == "ai":
                self.phase_timer = max(1, min(90, self.phase_timer))
            state = self._snapshot_locked()
        await self._broadcast_state(state)
        return state

    async def stop(self) -> SimulationState:
        async with self._lock:
            self.is_running = False
            state = self._snapshot_locked()
        await self._broadcast_state(state)
        return state

    async def set_speed(self, multiplier: float) -> SimulationState:
        async with self._lock:
            self.speed = max(0.5, min(4.0, multiplier))
            state = self._snapshot_locked()
        await self._broadcast_state(state)
        return state

    async def get_comparison_stats(self) -> ComparisonStats:
        async with self._lock:
            fixed_avg   = self._avg_wait(self.wait_times["fixed"])
            ai_avg      = self._avg_wait(self.wait_times["ai"])
            improvement = 0
            if fixed_avg > 0 and ai_avg > 0:
                improvement = int(round(((fixed_avg - ai_avg) / fixed_avg) * 100))
            return ComparisonStats(
                mode=self.mode,
                avgFixedWait=fixed_avg,
                avgAIWait=ai_avg,
                fixedSamples=len(self.wait_times["fixed"]),
                aiSamples=len(self.wait_times["ai"]),
                improvementPct=improvement,
                totalPassed=self.total_passed,
                throughputPerMinute=round(
                    (self.total_passed / max(1, self.sim_time)) * 60.0, 2
                ),
                avgQueueDepth=round(
                    sum(self.queues.values()) / max(1, len(self.queues)), 2
                ),
                queueStability=max(
                    0,
                    100 - (max(self.queues.values()) - min(self.queues.values())) * 4
                ),
            )

    async def get_queue_history(self) -> QueueHistoryResponse:
        async with self._lock:
            return QueueHistoryResponse(history=list(self.history))

    def _reset_runtime_state(self) -> None:
        self.is_running          = False
        self.active_dir          = "north"
        self.dir_index           = 0
        self.signal_state        = "green"
        self.phase_timer         = 30
        self.queues              = self._create_initial_queues(self.peak_hour, self.heavy_north)
        self.lane_queues         = self._create_initial_lane_queues(self.queues)
        self.total_passed        = 0
        self.wait_times          = {"fixed": [], "ai": []}
        self.vehicles            = []
        self.intersections       = self._create_intersections()
        self.sim_time            = 0
        self.history             = []
        self._frame              = 0
        self._vehicle_id         = 0
        self._second_accumulator = 0.0
        # Датасетийн параметрүүд хэвээр хадгална — reset хийхэд устгахгүй
        self._refresh_green_times()

    # ═══════════════════════════════════════════════════════════
    # WEBSOCKET
    # ═══════════════════════════════════════════════════════════

    from fastapi import WebSocketDisconnect

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._subscribers.add(websocket)

        try:
            state = await self.get_state()
            await websocket.send_json(state.model_dump())
        except Exception as e:
            print(f"[ws] initial send error: {e}")
            self._subscribers.discard(websocket)


    async def disconnect(self, websocket: WebSocket) -> None:
        self._subscribers.discard(websocket)


    async def listen(self, websocket: WebSocket) -> None:
        await self.connect(websocket)

        try:
            while True:
                await websocket.receive_text()

        except WebSocketDisconnect:
            print("[ws] disconnected")

        except Exception as e:
            print(f"[ws] listen error: {e}")

        finally:
            await self.disconnect(websocket)


    async def _broadcast_state(self, state: SimulationState | None = None) -> None:
        if not self._subscribers:
            return

        if state is None:
            state = await self.get_state()

        payload = state.model_dump()
        stale: list[WebSocket] = []

        for ws in list(self._subscribers):
            try:
                await ws.send_json(payload)
            except Exception:
                stale.append(ws)

        for ws in stale:
            self._subscribers.discard(ws)