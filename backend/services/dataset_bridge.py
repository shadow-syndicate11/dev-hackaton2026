"""
dataset_bridge.py
═════════════════════════════════════════════════════════════════════
Датасет ↔ Симулятор холболтын гүүр

Зорилго:
  - UBTrafficDatasetLoader-оос мэдээлэл аваад
  - TrafficSimulator-т шууд ачаалах
  - Датагаас машин үүсгэх, дараалал тохируулах, цаг агаар нөлөөлөх

Байрлал: backend/services/dataset_bridge.py
"""

from __future__ import annotations

import random
from typing import Any

# ── Датасет ────────────────────────────────────────────────────────
from backend.services.dataset_loader import (
    UBTrafficDatasetLoader,
    IntersectionSnapshot,
    LANE_TO_DIRECTION,
    WEATHER_SPEED_FACTOR,
    WEATHER_QUEUE_MULTIPLIER,
)

# ── AI контроллер ───────────────────────────────────────────────────
from backend.services.ai_controller import (
    DIRECTIONS,
    LANE_X,
    LANE_Y,
    calculate_green_time,
    VEHICLE_WEIGHTS,
    MIN_GREEN,
    MAX_GREEN,
    CYCLE_TOTAL,
    PEAK_MULTIPLIER,
)


# ═══════════════════════════════════════════════════════════════════
# ТОГТМОЛ УТГУУД
# ═══════════════════════════════════════════════════════════════════

VEHICLE_TYPE_WEIGHTS = {
    "car":   [0.70, 0.15, 0.15],  # straight, left, right
    "bus":   [0.80, 0.10, 0.10],  # автобус шулуун явна
    "truck": [0.75, 0.12, 0.13],
}

# Симуляторын canvas хэмжээ
CANVAS_W = 500.0
CANVAS_H = 400.0
SPAWN_OFFSET = 40.0
CENTER_X = CANVAS_W / 2
CENTER_Y = CANVAS_H / 2

# Машин үүсгэх анхны байрлал
SPAWN_POSITIONS: dict[str, dict[str, float]] = {
    "north": {"x": (LANE_X["north"][0] + LANE_X["north"][1]) / 2, "y": -SPAWN_OFFSET},
    "south": {"x": (LANE_X["south"][0] + LANE_X["south"][1]) / 2, "y": CANVAS_H + SPAWN_OFFSET},
    "east":  {"x": -SPAWN_OFFSET,             "y": (LANE_Y["east"][0] + LANE_Y["east"][1]) / 2},
    "west":  {"x": CANVAS_W + SPAWN_OFFSET,   "y": (LANE_Y["west"][0] + LANE_Y["west"][1]) / 2},
}

DIRECTION_ANGLE = {
    "north": 3.14159,
    "south": 0.0,
    "east":  1.5708,
    "west":  -1.5708,
}

VEHICLE_DYNAMICS = {
    "car":       {"cruise": 42.0, "accel": 58.0, "brake": 92.0},
    "truck":     {"cruise": 32.0, "accel": 38.0, "brake": 70.0},
    "bus":       {"cruise": 30.0, "accel": 34.0, "brake": 64.0},
    "emergency": {"cruise": 48.0, "accel": 70.0, "brake": 100.0},
}

VEHICLE_COLORS = {
    "north": "#ff6d00",
    "south": "#00e5ff",
    "east":  "#ffd600",
    "west":  "#c653ff",
}


# ═══════════════════════════════════════════════════════════════════
# ҮНДСЭН КЛАСС
# ═══════════════════════════════════════════════════════════════════

class DatasetBridge:
    """
    DatasetBridge нь UBTrafficDatasetLoader-оос snapshot авч,
    TrafficSimulator-т шаардлагатай бүх утгыг бэлтгэнэ.

    Хэрэглэх жишээ:
        bridge = DatasetBridge(loader)
        config = bridge.build_simulator_config(intersection_id=1, use_peak=True)
        await simulator.apply_dataset_config(config)
    """

    def __init__(self, loader: UBTrafficDatasetLoader) -> None:
        self._loader = loader
        self._rng = random.Random()
        self._vehicle_id = 0

    # ─────────────────────────────────────────────────────────────
    # СИМУЛЯТОРТ ШИЛЖҮҮЛЭХ БҮРЭН ТОХИРГОО
    # ─────────────────────────────────────────────────────────────

    def build_simulator_config(
        self,
        intersection_id: int,
        use_peak: bool = False,
        use_heaviest: bool = False,
        mode: str = "fixed",
    ) -> dict[str, Any]:
        """
        Датасетийн нэг intersection-н snapshot-г уншаад
        simulator.apply_dataset_config()-д шилжүүлэх dict буцаана.

        Args:
            intersection_id: 1–35
            use_peak:    Оргил ачааллын snapshot ашиглах
            use_heaviest: Хамгийн хүнд ачааллын snapshot ашиглах
            mode:        "fixed" | "ai"
        """
        cfg = self._loader.get_simulator_config(
            intersection_id,
            use_peak_data=use_peak,
            use_heaviest=use_heaviest,
        )
        if not cfg:
            return self._fallback_config(mode)

        snapshot = self._get_snapshot(intersection_id, use_peak, use_heaviest)
        if snapshot is None:
            return self._fallback_config(mode)

        # ── Дараалал ─────────────────────────────────────────────
        queues = cfg.get("queues", snapshot.direction_queues)

        # ── Машины тоогоор тооцсон AI ногоон гэрэл ─────────────
        vehicle_type_counts = self._build_vehicle_type_counts(snapshot)
        if mode == "ai":
            green_times = calculate_green_time(
                vehicle_counts=snapshot.direction_vehicle_counts,
                is_peak_hour=snapshot.is_peak_hour,
                bus_directions=snapshot.bus_directions,
                vehicle_type_counts=vehicle_type_counts,
            )
        else:
            # Тогтмол: датасетийн бодит green_sec-г ашиглана
            green_times = {
                d: snapshot.green_times.get(d, 30)
                for d in DIRECTIONS
            }

        # ── Машин үүсгэх параметрүүд ─────────────────────────────
        weather_factor = cfg.get("weather_factor", 1.0)
        spawn_chance   = cfg.get("spawn_chance", 0.45)
        max_vehicles   = cfg.get("max_vehicles", 56)
        arrival_rate   = cfg.get("arrival_rate", 0.30)
        discharge_rate = cfg.get("discharge_rate", 2)

        # ── Анхны машин жагсаалт датаас үүсгэнэ ─────────────────
        initial_vehicles = self._spawn_initial_vehicles(
            snapshot=snapshot,
            queues=queues,
            weather_factor=weather_factor,
        )

        return {
            # Үндсэн тохиргоо
            "mode":              mode,
            "peak_hour":         snapshot.is_peak_hour,
            "heavy_north":       queues.get("north", 0) == max(queues.values()),
            "bus_directions":    snapshot.bus_directions,
            "emergency_directions": [],

            # Дараалал
            "queues":            queues,
            "lane_queues":       self._split_to_lane_queues(queues),

            # Гэрлэн дохио
            "green_times":       green_times,
            "phase_timer":       max(green_times.values(), default=30),

            # Машин
            "vehicles":          initial_vehicles,
            "max_vehicles":      max_vehicles,

            # Физик
            "spawn_chance":      spawn_chance,
            "arrival_rate":      arrival_rate,
            "discharge_rate":    discharge_rate,
            "weather_factor":    weather_factor,
            "weather":           snapshot.weather,

            # Мета
            "intersection_id":   intersection_id,
            "intersection_name": snapshot.intersection_name,
            "district":          snapshot.district,
            "timestamp":         str(snapshot.timestamp),
            "congestion_index":  snapshot.avg_congestion_index,
            "congestion_tier":   snapshot.congestion_tier,
            "load_factors":      snapshot.get_load_factors(),

            # KPI тооцоолол
            "kpi_baseline": {
                "avg_queue":      sum(queues.values()) / max(len(queues), 1),
                "congestion":     snapshot.avg_congestion_index,
                "fixed_green":    list(snapshot.green_times.values())[0] if snapshot.green_times else 30,
                "ai_green":       int(sum(green_times.values()) / max(len(green_times), 1)),
            }
        }

    # ─────────────────────────────────────────────────────────────
    # SIMULATOR APPLY METHOD (simulator.py-д нэмэх код)
    # ─────────────────────────────────────────────────────────────

    def get_apply_patch(self, config: dict[str, Any]) -> dict[str, Any]:
        """
        TrafficSimulator.apply_dataset_config()-д шилжүүлэх
        хялбарчилсан dict буцаана.
        """
        return {
            "mode":                   config["mode"],
            "peak_hour":              config["peak_hour"],
            "heavy_north":            config["heavy_north"],
            "bus_directions":         config["bus_directions"],
            "emergency_directions":   config["emergency_directions"],
            "queues":                 config["queues"],
            "lane_queues":            config["lane_queues"],
            "green_times":            config["green_times"],
            "phase_timer":            config["phase_timer"],
            "vehicles":               config["vehicles"],
            "spawn_chance":           config["spawn_chance"],
            "arrival_rate":           config["arrival_rate"],
            "discharge_rate":         config["discharge_rate"],
            "weather_factor":         config["weather_factor"],
        }

    # ─────────────────────────────────────────────────────────────
    # АНХНЫ МАШИН ҮҮСГЭХ
    # ─────────────────────────────────────────────────────────────

    def _spawn_initial_vehicles(
        self,
        snapshot: IntersectionSnapshot,
        queues: dict[str, int],
        weather_factor: float = 1.0,
    ) -> list[dict[str, Any]]:
        """
        Датасетийн queue_length-г үндэслэн анхны машинуудыг үүсгэнэ.
        Машин бүрийн vehicle_type нь датасетийн vehicle_type-аас авна.
        """
        vehicles: list[dict[str, Any]] = []

        # Чиглэл бүрийн давамгай machine type-ийг датаас авна
        dir_vtypes = self._dominant_vehicle_type_per_direction(snapshot)

        for direction in DIRECTIONS:
            queue_size = min(queues.get(direction, 0), 20)  # canvas-д багтах хэмжээ
            if queue_size <= 0:
                continue

            vtype_weights = self._lane_vehicle_type_distribution(snapshot, direction)
            spawn = SPAWN_POSITIONS[direction]
            lane_x_list = LANE_X.get(direction, [spawn["x"]])
            lane_y_list = LANE_Y.get(direction, [spawn["y"]])

            for i in range(queue_size):
                self._vehicle_id += 1
                lane_idx = i % 2
                vtype = self._rng.choices(
                    list(vtype_weights.keys()),
                    weights=list(vtype_weights.values()),
                    k=1
                )[0]
                turn_weights = VEHICLE_TYPE_WEIGHTS.get(vtype, [0.6, 0.2, 0.2])
                turn = self._rng.choices(
                    ["straight", "left", "right"],
                    weights=turn_weights, k=1
                )[0]

                # Spawn байрлал — зогсоолын шугамын ард дараалаад байна
                offset = (i // 2 + 1) * 22  # 22px зайтай
                if direction == "north":
                    x = LANE_X["north"][lane_idx]
                    y = 130.0 - offset          # зогсоолын шугамын хойд талд
                elif direction == "south":
                    x = LANE_X["south"][lane_idx]
                    y = 270.0 + offset
                elif direction == "east":
                    x = 180.0 - offset
                    y = LANE_Y["east"][lane_idx]
                else:  # west
                    x = 320.0 + offset
                    y = LANE_Y["west"][lane_idx]

                # Цаг агаарын нөлөөгөөр хурдыг бууруулна
                cruise = VEHICLE_DYNAMICS[vtype]["cruise"] * weather_factor

                vehicles.append({
                    "id":           self._vehicle_id,
                    "dir":          direction,
                    "type":         vtype,
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
                    "waiting":      True,       # зогсоолын шугамд хүлээж байна
                    "color":        VEHICLE_COLORS[direction],
                    # Датасетаас авсан нэмэлт мэдээлэл
                    "fromDataset":  True,
                    "weatherFactor": weather_factor,
                })

        return vehicles

    # ─────────────────────────────────────────────────────────────
    # ТУСЛАХ МЕТОДУУД
    # ─────────────────────────────────────────────────────────────

    def _get_snapshot(
        self,
        intersection_id: int,
        use_peak: bool,
        use_heaviest: bool,
    ) -> IntersectionSnapshot | None:
        if use_heaviest:
            s = self._loader.get_heaviest_peak_snapshot(intersection_id)
            return s or self._loader.get_latest_peak_snapshot(intersection_id)
        if use_peak:
            return self._loader.get_latest_peak_snapshot(intersection_id)
        return self._loader.get_latest_snapshot(intersection_id)

    def _build_vehicle_type_counts(
        self, snapshot: IntersectionSnapshot
    ) -> dict[str, dict[str, int]]:
        """Чиглэл бүрийн vehicle_type тоог dict болгоно."""
        result: dict[str, dict[str, int]] = {d: {} for d in DIRECTIONS}
        for lane in snapshot.lanes:
            d = lane.direction
            vt = lane.vehicle_type
            result[d][vt] = result[d].get(vt, 0) + lane.vehicle_count
        return result

    def _dominant_vehicle_type_per_direction(
        self, snapshot: IntersectionSnapshot
    ) -> dict[str, str]:
        counts: dict[str, dict[str, int]] = {d: {} for d in DIRECTIONS}
        for lane in snapshot.lanes:
            d = lane.direction
            vt = lane.vehicle_type
            counts[d][vt] = counts[d].get(vt, 0) + lane.vehicle_count
        return {
            d: max(vtmap, key=vtmap.get) if vtmap else "car"
            for d, vtmap in counts.items()
        }

    def _lane_vehicle_type_distribution(
        self, snapshot: IntersectionSnapshot, direction: str
    ) -> dict[str, float]:
        """Тухайн чиглэлд ямар машин хэдэн хувь байгааг буцаана."""
        dist: dict[str, float] = {}
        for lane in snapshot.lanes:
            if lane.direction == direction:
                vt = lane.vehicle_type
                dist[vt] = dist.get(vt, 0.0) + float(lane.vehicle_count)
        total = sum(dist.values()) or 1.0
        if not dist:
            return {"car": 1.0}
        return {vt: cnt / total for vt, cnt in dist.items()}

    def _split_to_lane_queues(self, queues: dict[str, int]) -> dict[str, int]:
        """direction → lane_0, lane_1 хуваана."""
        result: dict[str, int] = {}
        for d, total in queues.items():
            result[f"{d}_0"] = total // 2
            result[f"{d}_1"] = total - total // 2
        return result

    def _fallback_config(self, mode: str) -> dict[str, Any]:
        """Датасет олдохгүй үед default утгуудыг буцаана."""
        queues = {"north": 9, "south": 7, "east": 6, "west": 7}
        green_times = {d: 30 for d in DIRECTIONS}
        return {
            "mode":               mode,
            "peak_hour":          False,
            "heavy_north":        False,
            "bus_directions":     [],
            "emergency_directions": [],
            "queues":             queues,
            "lane_queues":        self._split_to_lane_queues(queues),
            "green_times":        green_times,
            "phase_timer":        30,
            "vehicles":           [],
            "max_vehicles":       56,
            "spawn_chance":       0.45,
            "arrival_rate":       0.30,
            "discharge_rate":     2,
            "weather_factor":     1.0,
            "weather":            "Clear",
            "intersection_id":    1,
            "intersection_name":  "Тодорхойгүй",
            "district":           "",
            "timestamp":          "",
            "congestion_index":   0.0,
            "congestion_tier":    "free",
            "load_factors":       {},
            "kpi_baseline":       {},
        }