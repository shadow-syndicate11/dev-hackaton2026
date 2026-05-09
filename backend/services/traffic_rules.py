"""
traffic_rules.py
================
Монгол Улсын Замын Хөдөлгөөний Дүрэм (2018 оны 239 дүгээр тогтоол) болон
МНС 4596:2007 "Замын тэмдэг, тэмдэглэл, гэрлэн дохио, хашилт,
чиглүүлэх хэрэгслүүдийг хэрэглэх дүрэм" стандартын заалтуудыг
симулятортой уялдуулсан зохицуулгын модуль.

Хэрэгжүүлсэн дүрмүүд:
── Замын хөдөлгөөний дүрэм (2018) ──────────────────────────────────────────
  • 4.2  — Онцгой тусгай дуут болон гэрлэн дохиотой тээврийн хэрэгслийн
            давуу эрх
  • 4.4  — Онцгой тусгай дохиотой тээврийн хэрэгсэлд зам тавьж өгөх
  • 8.9  — Гэрлэн дохионы утга (ногоон/шар/улаан/анивчсан)
  • 8.18 — Хориглосон дохиогоор зогсох газар (зогсох шугамын өмнө)
  • 8.19 — Шар дохио асахад огцом тоормослохгүй аюулгүй үргэлжлүүлэх
  • 10.2 — Хөдөлгөөн эхлэхдээ бусадад зам тавьж өгөх
  • 10.9 — Зүүн тийш / буцаж эргэхдээ зам тавьж өгөх
  • 11.14— Хөдөлгөөний хурдаас хамаарсан аюулгүй зай
  • 12.4 — Хурдны дээд хязгаар (суурин: 60, гадна: 80, тууш: 100 км/ц)
  • 12.5 — Тусгай тооны тээврийн хэрэгслийн хурдны хязгаар
  • 12.6г— Шалтгаангүй хэт удаан явахыг хориглоно
  • 13.2 — Гүйцэж түрүүлэхийг хориглох нөхцөлүүд
  • 13.4 — Гүйцэж түрүүлэхийг хориглох газрууд
  • 14.8 — Түр зогсохыг хориглох газрууд
  • 15.8 — Адил замын уулзварт баруун гараас ирсэнд зам тавих
  • 15.9 — Гол/туслах замын уулзварт туслах замаас зам тавих
  • 16.1 — Явган зорчигчийн зохицуулдаггүй гарцад зам тавих

── МНС 4596:2007 "Замын гэрлэн дохиог хэрэглэх дүрэм" ─────────────────────
  • 6.2.1— Гэрлэн дохиог байрлуулах зарчим (100 м-ийн зайнаас харагдах)
  • 6.2.3— Т.3, Т.4 дохио — зөвхөн чиглэлийн хөдөлгөөнийг зохицуулна
  • 6.2.9— Т.1-Т.4, Я.1, Я.2 дохионы дараах дөрвөн нөхцөлд байрлуулна
  • 6.3.4— Зорчих хэсгийн хажуугийн байрлал (зах хүртэл 0.5-2 м)
  • 6.4.1— Тухайн нэг бүлэг гэрлэн дохионууд харилцан хамаарах горимоор
  • 6.4.2— Т.1-Т.5 гэрлэн дохионы дараалал:
            улаан → улаан ба шар → ногоон → шар → улаан
  • 6.4.3— Ногоон гэрэл нь 3 секундын дотор унтраахаас өмнө анивчна
            (МНС 6.4.3: Я.1 ба Я.2 горим заавал)
  • 6.4.4— Замын тухайн хэсгийн хөдөлгөөний эрчим стандартын 6.2.9
            дүгээр зүйлийн 1 ба 2 дугаар нөхцөлийн утгаас 50%-аас доош
            болтол буурсан үед шар гэрлийн горимд шилжинэ
  • 6.4.5— Т.6.х дохио хос гэрэл нь зэлхлэн асаж унтрах горимоор,
            Т.6 ба Т.7 дохионы гэрэл нь анивчих горимоор ажилладаг
  • 6.4.6— Я.1 ба Я.2 гэрлэн дохио:
            улаан → ногоон → улаан дараалалтай
  • 6.3.7— Т.1, Т.3 гэрлэн дохионы зорчих хэсгийн хажуу байрлал,
            нэмэлт дохионы зүүн/баруун чиглэлийн зохицуулалт
  • 6.3.8— Уулзварын эрхийн хажуугийн байрлал давтан тавих нөхцөл
  • 6.3.9— Стандартын 6.3.8 шаардлагыг мөрдөхгүй тохиолдолд
            тухайн гэрлэн дохиог зорчих хэсгийн дээр байрлуулна
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

# ─────────────────────────────────────────────────────────────────────────────
# ТОГТМОЛУУД
# ─────────────────────────────────────────────────────────────────────────────
# Симулятор: PHYSICS_HZ=12, car cruise ≈ 42 px/sec ≈ 60 км/ц
KMH_TO_PX_SEC: float = 42.0 / 60.0   # 0.7 px/sec per km/h

# МНС 6.4.2 — гэрлэн дохионы горим дахь хугацаа (секунд)
MNS_YELLOW_SEC:        int = 3    # 6.4.2 шар гэрлийн хугацаа
MNS_ALL_RED_SEC:       int = 2    # бүх улаан гэрлийн хугацаа
MNS_YELLOW_RED_SEC:    int = 2    # улаан + шар хамт асах хугацаа (6.4.2)
MNS_GREEN_FLASH_SEC:   int = 3    # 6.4.3 ногоон гэрэл анивчих хугацаа

# МНС 6.4.4 — хөдөлгөөний эрчим 50%-аас доош бол шар горимд шилжих
MNS_LOW_VOLUME_THRESHOLD: float = 0.50   # 50%

# МНС 6.2.9 — гэрлэн дохиог байрлуулах нөхцөлийн нормативын утгууд
# (нэг чиглэлд нэг зурвасаар хөдөлгөөний эрчим нэгж/цаг)
MNS_VOLUME_TABLE: dict[str, list[int]] = {
    # "гол зам нэг чиглэл / туслах зам нэг чиглэл": [утга, утга, ...]
    "1_lane_1_lane":    [750, 670, 580, 500, 410, 380],
    "2plus_lane_1_lane":[900, 800, 700, 600, 500, 400],
    "2plus_lane_2plus": [900, 825, 750, 675, 600, 525, 480],
}

# ─────────────────────────────────────────────────────────────────────────────
# ENUM-УУД
# ─────────────────────────────────────────────────────────────────────────────

class ZoneType(Enum):
    """Замын бүс — ЗХД 12.4, 12.5 заалт"""
    RESIDENTIAL = auto()   # суурин газар: 60 км/ц
    RURAL       = auto()   # суурин газрын гадна: 80 км/ц
    HIGHWAY     = auto()   # тууш зам: 100 км/ц
    SCHOOL_ZONE = auto()   # сургуулийн орчим: 20 км/ц
    DISTRICT    = auto()   # хороолол: 20 км/ц


SPEED_LIMITS: dict[ZoneType, float] = {
    ZoneType.RESIDENTIAL: 60.0,
    ZoneType.RURAL:       80.0,
    ZoneType.HIGHWAY:    100.0,
    ZoneType.SCHOOL_ZONE: 20.0,
    ZoneType.DISTRICT:    20.0,
}

SPEED_LIMIT_BUS_RESIDENTIAL = 50.0   # 12.5а
SPEED_LIMIT_BUS_RURAL       = 70.0   # 12.5а
SPEED_LIMIT_BUS_HIGHWAY     = 80.0   # 12.5а
SPEED_LIMIT_CHILDREN        = 50.0   # 12.5в
SPEED_LIMIT_TOWING          = 40.0   # 12.5г


class SignalPhase(Enum):
    """
    МНС 6.4.2 — Т.1-Т.5 гэрлэн дохионы горим:
      YELLOW_RED   = улаан + шар хамт асаж байна (хөдөлгөөн эхлэхийн өмнө)
      GREEN_FLASH  = ногоон гэрэл анивчиж байна (6.4.3 — зогсох бэлтгэл)
      YELLOW       = шар гэрэл (хөдөлгөөн зогсох анхааруулга)
      ALL_RED      = бүх чиглэл хориглогдсон (чиглэл солих завсар)

    ЗХД 8.9 — утга:
      GREEN        = ногоон: хөдөлгөөн зөвшөөрөгдөнө
      RED          = улаан: хориглоно
      FLASHING_YEL = анивчсан шар: зохицуулгагүй, болгоомжтой
    """
    GREEN        = "green"         # 8.9а, МНС 6.4.2
    GREEN_FLASH  = "green_flash"   # 6.4.3 — ногоон анивчна (3 сек)
    YELLOW       = "yellow"        # 8.9б, МНС 6.4.2 — шар (3 сек)
    YELLOW_RED   = "yellow_red"    # МНС 6.4.2 — улаан+шар хамт (2 сек)
    ALL_RED      = "all_red"       # МНС 6.4.2 — бүх улаан (2 сек)
    RED          = "red"           # 8.9г — улаан
    FLASHING_YEL = "flashing"      # 8.9в — анивчсан шар (зохицуулгагүй)


# МНС 6.4.2 — Т.1-Т.5 дохионы бүрэн горимын дараалал
FULL_SIGNAL_SEQUENCE: list[str] = [
    SignalPhase.RED.value,          # улаан
    SignalPhase.YELLOW_RED.value,   # улаан + шар (2 сек) — МНС 6.4.2
    SignalPhase.GREEN.value,        # ногоон
    SignalPhase.GREEN_FLASH.value,  # ногоон анивч (3 сек) — МНС 6.4.3
    SignalPhase.YELLOW.value,       # шар (3 сек) — МНС 6.4.2
    SignalPhase.ALL_RED.value,      # бүх улаан (2 сек)
]

# МНС 6.4.6 — Я.1, Я.2 явган зорчигчийн гэрлэн дохионы дараалал
PEDESTRIAN_SIGNAL_SEQUENCE: list[str] = [
    SignalPhase.RED.value,
    SignalPhase.GREEN.value,
    SignalPhase.GREEN_FLASH.value,  # 6.4.3 — анивчна
    SignalPhase.RED.value,
]

# МНС 6.4.5 — Т.6.х хос гэрэл (зэлхлэн асаж унтрах) болон
#              Т.6, Т.7 дохио (анивчих горим)
ARROW_SIGNAL_MODES: dict[str, str] = {
    "T6x": "alternating",   # зэлхлэн асаж унтрах
    "T6":  "flashing",      # анивчих
    "T7":  "flashing",      # анивчих
}


class VehicleClass(Enum):
    """Тээврийн хэрэгслийн ангилал — ЗХД 2.1 заалт"""
    A         = "motorcycle"
    B         = "car"
    C         = "truck"
    D         = "bus"
    M         = "tractor"
    EMERGENCY = "emergency"


# МНС 6.3.7 — нэмэлт дохионы чиглэл
class AdditionalSignalDirection(Enum):
    """МНС 6.3.7 — Т.1, Т.3 нэмэлт дохионы чиглэл"""
    LEFT          = "left"
    RIGHT         = "right"
    STRAIGHT      = "straight"
    LEFT_RIGHT    = "left_right"
    ALL           = "all"


# ─────────────────────────────────────────────────────────────────────────────
# ДАТАКЛАССУУД
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SignalTimingMNS:
    """
    МНС 6.4.2 — Т.1-Т.5 гэрлэн дохионы горимын бүрэн хугацааны бүтэц.
    Симуляторын phase_timer-ийг тооцоолоход ашиглана.
    """
    green_sec:      int         # ногоон гэрлийн хугацаа (AI тооцооноос)
    green_flash_sec: int = MNS_GREEN_FLASH_SEC    # 6.4.3: 3 сек
    yellow_sec:      int = MNS_YELLOW_SEC          # 6.4.2: 3 сек
    all_red_sec:     int = MNS_ALL_RED_SEC         # 6.4.2: 2 сек
    yellow_red_sec:  int = MNS_YELLOW_RED_SEC      # 6.4.2: 2 сек

    @property
    def total_cycle_sec(self) -> int:
        """Нэг бүрэн цикл = улаан + улаан+шар + ногоон + ногоон анивч + шар + бүх улаан"""
        return (
            self.green_sec
            + self.green_flash_sec
            + self.yellow_sec
            + self.all_red_sec
            + self.yellow_red_sec
        )

    def get_phase_duration(self, phase: str) -> int:
        """Тухайн фазын хугацааг буцаана"""
        return {
            SignalPhase.GREEN.value:       self.green_sec,
            SignalPhase.GREEN_FLASH.value: self.green_flash_sec,
            SignalPhase.YELLOW.value:      self.yellow_sec,
            SignalPhase.ALL_RED.value:     self.all_red_sec,
            SignalPhase.YELLOW_RED.value:  self.yellow_red_sec,
            SignalPhase.RED.value:         0,   # улааны хугацаа нь нөгөө чиглэлийн цикл
        }.get(phase, 0)


@dataclass
class AdditionalSignalMNS:
    """
    МНС 6.3.7 — Т.1, Т.3 гэрлэн дохионд нэмэлт дохио
    Нэмэлт дохио нь үндсэн дохиотой хамт ажиллана.
    """
    direction:         AdditionalSignalDirection
    active_with_phase: str          # ямар үндсэн фазтай хамт асах вэ
    is_flashing:       bool = False # МНС 6.4.5: анивчих горим


@dataclass
class TrafficRuleContext:
    """Тээврийн хэрэгслийн нөхцөл байдал — _move_vehicle дотор ашиглана."""
    vehicle_id:           int
    direction:            str       # "north"|"south"|"east"|"west"
    vehicle_type:         str       # "car"|"bus"|"truck"|"emergency"
    x:                    float
    y:                    float
    speed:                float     # pixel/sec
    lane:                 int
    turn:                 str       # "straight"|"left"|"right"
    turn_progress:        float
    signal_phase:         str       # SignalPhase.value
    active_dir:           str       # "north"|"east"
    zone_type:            ZoneType = ZoneType.RESIDENTIAL
    is_carrying_children: bool = False
    is_towing:            bool = False
    is_emergency:         bool = False
    is_yielding:          bool = False
    waiting_ticks:        int  = 0
    nearby_pedestrians:   list[dict] = field(default_factory=list)
    same_lane_vehicles:   list[dict] = field(default_factory=list)
    opposite_vehicles:    list[dict] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# МНС 6.4.2 — ГЭРЛЭН ДОХИОНЫ ГОРИМ ДАРААЛАЛ ТООЦООЛОХ
# ─────────────────────────────────────────────────────────────────────────────

def get_next_signal_phase(current_phase: str) -> tuple[str, int]:
    """
    МНС 6.4.2 — Т.1-Т.5 дохионы горимын дараагийн фаз болон хугацааг буцаана.

    Дараалал:
      RED → YELLOW_RED(2с) → GREEN(AI тооц) → GREEN_FLASH(3с) → YELLOW(3с) → ALL_RED(2с) → RED

    Returns:
        (next_phase, duration_seconds)
    """
    transitions: dict[str, tuple[str, int]] = {
        SignalPhase.RED.value:         (SignalPhase.YELLOW_RED.value,  MNS_YELLOW_RED_SEC),
        SignalPhase.YELLOW_RED.value:  (SignalPhase.GREEN.value,       0),     # хугацааг AI тооцооноос авна
        SignalPhase.GREEN.value:       (SignalPhase.GREEN_FLASH.value, MNS_GREEN_FLASH_SEC),
        SignalPhase.GREEN_FLASH.value: (SignalPhase.YELLOW.value,      MNS_YELLOW_SEC),
        SignalPhase.YELLOW.value:      (SignalPhase.ALL_RED.value,     MNS_ALL_RED_SEC),
        SignalPhase.ALL_RED.value:     (SignalPhase.RED.value,         0),     # нөгөө чиглэл эхлэнэ
    }
    return transitions.get(current_phase, (SignalPhase.RED.value, 0))


def can_proceed_on_phase(phase: str) -> bool:
    """
    МНС 6.4.2, ЗХД 8.9 — тухайн фазд хөдөлгөөн үргэлжлүүлж болох эсэх.

    Зөвшөөрөгдсөн фазууд:
      GREEN      — бүрэн хөдөлгөөн
      GREEN_FLASH — анивчих ногоон: зогсох боломжгүй бол үргэлжлүүлнэ
      YELLOW_RED  — хөдөлгөөн эхлэхэд бэлдэж байна (зогссон машинд)
    Хориглогдсон:
      YELLOW, ALL_RED, RED
    """
    return phase in {
        SignalPhase.GREEN.value,
        SignalPhase.GREEN_FLASH.value,
        SignalPhase.YELLOW_RED.value,
    }


def is_stop_required_on_phase(phase: str) -> bool:
    """МНС 6.4.2, ЗХД 8.9г — заавал зогсох шаардлагатай фаз"""
    return phase in {
        SignalPhase.RED.value,
        SignalPhase.ALL_RED.value,
    }


def mns_signal_timing(green_sec: int) -> SignalTimingMNS:
    """
    МНС 6.4.2 — тухайн ногоон гэрлийн хугацаанд суурилсан
    бүрэн горимын хугацааны бүтцийг буцаана.
    """
    return SignalTimingMNS(green_sec=max(12, green_sec))


# ─────────────────────────────────────────────────────────────────────────────
# МНС 6.4.4 — ХӨДӨЛГӨӨНИЙ ЭРЧИМ БАГА ҮЕИЙН ШАР ГОРИМ
# ─────────────────────────────────────────────────────────────────────────────

def should_switch_to_flashing_yellow(
    current_volume: int,
    design_volume: int,
    lane_config: str = "1_lane_1_lane",
) -> bool:
    """
    МНС 6.4.4 — Замын тухайн хэсгийн хөдөлгөөний эрчим стандартын 6.2.9
    дүгээр зүйлийн 1 ба 2 дугаар нөхцөлийн утгаас 50%-аас доош болтол
    буурсан үед Т.1-Т.5 гэрлэн дохионуудыг шар гэрлийн горимд шилжүүлнэ.

    Args:
        current_volume:  одоогийн хөдөлгөөний эрчим (нэгж/цаг)
        design_volume:   МНС 6.2.9-ийн хүснэгтийн норматив утга
        lane_config:     "1_lane_1_lane" | "2plus_lane_1_lane" | "2plus_lane_2plus"
    """
    threshold = design_volume * MNS_LOW_VOLUME_THRESHOLD
    return current_volume < threshold


def get_mns_design_volume(
    lane_config: str,
    tier_index: int = 0,
) -> int:
    """
    МНС 6.2.9 — 5 дугаар хүснэгтийн норматив утгыг буцаана.
    tier_index: 0=хамгийн өндөр норматив, сүүлийнх=хамгийн бага
    """
    table = MNS_VOLUME_TABLE.get(lane_config, MNS_VOLUME_TABLE["1_lane_1_lane"])
    idx = max(0, min(tier_index, len(table) - 1))
    return table[idx]


# ─────────────────────────────────────────────────────────────────────────────
# МНС 6.3.7 — НЭМЭЛТ ДОХИОНЫ ЛОГИК
# ─────────────────────────────────────────────────────────────────────────────

def get_additional_signal_state(
    additional: AdditionalSignalMNS,
    current_phase: str,
    is_low_volume: bool = False,
) -> dict[str, Any]:
    """
    МНС 6.3.7 — Т.1, Т.3 гэрлэн дохионы нэмэлт дохионы байдлыг буцаана.

    МНС 6.4.5 — Т.6.х хос гэрэл зэлхлэн, Т.6/Т.7 анивчих горимоор.

    Returns:
        {
          "active":    bool — нэмэлт дохио асаж байна уу,
          "flashing":  bool — анивчиж байна уу,
          "direction": str  — зөвшөөрөгдөж буй чиглэл,
          "reason":    str,
        }
    """
    # Нэмэлт дохио идэвхтэй болох нөхцөл
    is_active = (current_phase == additional.active_with_phase)

    # МНС 6.4.4: хөдөлгөөний эрчим бага үед нэмэлт дохио анивчина
    is_flashing = additional.is_flashing or is_low_volume

    return {
        "active":    is_active,
        "flashing":  is_flashing and is_active,
        "direction": additional.direction.value,
        "reason": (
            f"МНС 6.3.7: нэмэлт дохио {additional.direction.value} "
            f"чиглэлд {'идэвхтэй' if is_active else 'идэвхгүй'}"
        ),
    }


def evaluate_additional_signal_for_vehicle(
    turn: str,
    additional_signals: list[AdditionalSignalMNS],
    current_phase: str,
) -> bool:
    """
    МНС 6.3.7 — Тээврийн хэрэгслийн эргэх чиглэлд нэмэлт дохио
    зөвшөөрөл өгч байна уу.

    True → тухайн чиглэлд нэмэлт дохиогоор явж болно
    (үндсэн дохио улаан байсан ч)
    """
    for sig in additional_signals:
        state = get_additional_signal_state(sig, current_phase)
        if not state["active"]:
            continue
        d = sig.direction
        if d == AdditionalSignalDirection.ALL:
            return True
        if turn == "left"     and d in (AdditionalSignalDirection.LEFT,
                                         AdditionalSignalDirection.LEFT_RIGHT):
            return True
        if turn == "right"    and d in (AdditionalSignalDirection.RIGHT,
                                         AdditionalSignalDirection.LEFT_RIGHT):
            return True
        if turn == "straight" and d == AdditionalSignalDirection.STRAIGHT:
            return True
    return False


# ─────────────────────────────────────────────────────────────────────────────
# 1. ХУРДНЫ ХЯЗГААР — ЗХД 12.4, 12.5, 12.6
# ─────────────────────────────────────────────────────────────────────────────

def get_speed_limit_px_sec(ctx: TrafficRuleContext) -> float:
    """
    ЗХД 12.4, 12.5 дагуу хурдны дээд хязгаарыг pixel/sec-ээр буцаана.
    """
    if ctx.is_emergency:
        # 4.2 — онцгой тусгай дохиотой: хязгаарлалтгүй
        return VEHICLE_CRUISE_EMERGENCY_PX_SEC

    if ctx.is_carrying_children:
        return SPEED_LIMIT_CHILDREN * KMH_TO_PX_SEC     # 12.5в: 50 км/ц

    if ctx.is_towing:
        return SPEED_LIMIT_TOWING * KMH_TO_PX_SEC       # 12.5г: 40 км/ц

    if ctx.vehicle_type == "bus":
        limits = {
            ZoneType.RESIDENTIAL: SPEED_LIMIT_BUS_RESIDENTIAL,
            ZoneType.RURAL:       SPEED_LIMIT_BUS_RURAL,
            ZoneType.HIGHWAY:     SPEED_LIMIT_BUS_HIGHWAY,
            ZoneType.SCHOOL_ZONE: 20.0,
            ZoneType.DISTRICT:    20.0,
        }
        return limits.get(ctx.zone_type, SPEED_LIMIT_BUS_RESIDENTIAL) * KMH_TO_PX_SEC

    return SPEED_LIMITS.get(ctx.zone_type, 60.0) * KMH_TO_PX_SEC


VEHICLE_CRUISE_EMERGENCY_PX_SEC: float = 48.0   # emergency cruise


def check_minimum_speed(ctx: TrafficRuleContext, dynamics_cruise: float) -> float:
    """12.6г — хэт удаан явахыг хориглоно. Cruise-н 15%."""
    if ctx.turn_progress > 0:
        return 0.0
    return dynamics_cruise * 0.15


# ─────────────────────────────────────────────────────────────────────────────
# 2. ГЭРЛЭН ДОХИОНЫ ЛОГИК — ЗХД 8.9, 8.18, 8.19 + МНС 6.4.2, 6.4.3
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SignalDecision:
    can_proceed:        bool
    must_stop:          bool
    safe_to_continue:   bool
    speed_limit_ratio:  float   # 0.0–1.0
    phase_label:        str     # МНС горимын нэр
    reason:             str


def evaluate_signal(
    ctx: TrafficRuleContext,
    stop_distance_px: float,
    current_speed: float,
    brake_decel: float,
    additional_signals: list[AdditionalSignalMNS] | None = None,
    is_low_volume: bool = False,
) -> SignalDecision:
    """
    ЗХД 8.9, 8.18, 8.19 болон МНС 6.4.2, 6.4.3 заалтын дагуу
    гэрлэн дохионы нөхцөлд ямар үйлдэл хийхийг тодорхойлно.

    МНС 6.4.2-ийн шинэ фазууд:
      YELLOW_RED  → улаан + шар хамт: зогссон машин явахад бэлдэнэ
      GREEN_FLASH → ногоон анивч: зогсохоос өмнө 3 секундын анхааруулга
    """
    phase = ctx.signal_phase
    add_sigs = additional_signals or []

    # 4.2 — онцгой тусгай дохиотой: бүх дохиог үл харгалзана
    if ctx.is_emergency:
        return SignalDecision(
            can_proceed=True, must_stop=False, safe_to_continue=True,
            speed_limit_ratio=1.0,
            phase_label="ОНЦГОЙ",
            reason="4.2: онцгой тусгай дохиотой — дохио үл харгалзана",
        )

    # МНС 6.4.4 — хөдөлгөөний эрчим бага: анивчсан шар горим
    if is_low_volume and phase not in (
        SignalPhase.RED.value, SignalPhase.ALL_RED.value
    ):
        return SignalDecision(
            can_proceed=True, must_stop=False, safe_to_continue=True,
            speed_limit_ratio=0.55,
            phase_label="ШАР_АНИВЧ (6.4.4)",
            reason="МНС 6.4.4: хөдөлгөөний эрчим 50%-аас доош — анивчсан шар горим",
        )

    # ── НОГООН ГЭРЭЛ (8.9а, МНС 6.4.2) ──────────────────────────────────
    if phase == SignalPhase.GREEN.value:
        return SignalDecision(
            can_proceed=True, must_stop=False, safe_to_continue=True,
            speed_limit_ratio=1.0,
            phase_label="НОГООН",
            reason="8.9а / МНС 6.4.2: ногоон дохио — хөдөлгөөн зөвшөөрөгдөнө",
        )

    # ── НОГООН АНИВЧ (МНС 6.4.3) ─────────────────────────────────────────
    if phase == SignalPhase.GREEN_FLASH.value:
        stopping_dist = (current_speed ** 2) / (2.0 * max(brake_decel, 1.0))
        can_stop = stopping_dist < stop_distance_px - 5.0
        if can_stop:
            # Зогсох боломжтой → зогсох бэлтгэл
            ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 60.0))
            return SignalDecision(
                can_proceed=False, must_stop=False, safe_to_continue=False,
                speed_limit_ratio=ratio * ratio,
                phase_label="НОГООН_АНИВЧ (6.4.3)",
                reason="МНС 6.4.3: ногоон гэрэл анивчиж байна — зогсох бэлтгэл хийнэ",
            )
        else:
            # Огцом зогсох боломжгүй → 8.19 дагуу үргэлжлүүлнэ
            return SignalDecision(
                can_proceed=True, must_stop=False, safe_to_continue=True,
                speed_limit_ratio=0.85,
                phase_label="НОГООН_АНИВЧ → ҮРГЭЛЖЛҮҮЛ",
                reason="МНС 6.4.3 / ЗХД 8.19: анивчих ногоон — огцом зогсох боломжгүй, үргэлжлүүлнэ",
            )

    # ── УЛААН + ШАР ХАМТ (МНС 6.4.2) ────────────────────────────────────
    if phase == SignalPhase.YELLOW_RED.value:
        # Зогссон машинд: явахад бэлдэж байгаа сигнал → хөдөлгөөнгүй хүлээ
        return SignalDecision(
            can_proceed=False, must_stop=True, safe_to_continue=False,
            speed_limit_ratio=0.0,
            phase_label="УЛААН+ШАР (6.4.2)",
            reason="МНС 6.4.2: улаан + шар хамт — ногоон гэрэлд бэлдэж байна, хөдлөхгүй",
        )

    # ── ШАР ГЭРЭЛ (8.9б, МНС 6.4.2) ─────────────────────────────────────
    if phase == SignalPhase.YELLOW.value:
        stopping_dist = (current_speed ** 2) / (2.0 * max(brake_decel, 1.0))
        can_stop_safely = stopping_dist < stop_distance_px - 5.0

        # Нэмэлт дохиогоор тухайн чиглэлд явж болох эсэх (МНС 6.3.7)
        if add_sigs and evaluate_additional_signal_for_vehicle(
            ctx.turn, add_sigs, phase
        ):
            return SignalDecision(
                can_proceed=True, must_stop=False, safe_to_continue=True,
                speed_limit_ratio=0.7,
                phase_label="ШАР + НЭМЭЛТ (6.3.7)",
                reason="МНС 6.3.7: шар дохио ч гэсэн нэмэлт дохиогоор тухайн чиглэлд явна",
            )

        if can_stop_safely:
            # 8.18 — зогсох шугамын өмнө зогсоно
            ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 58.0))
            return SignalDecision(
                can_proceed=False, must_stop=True, safe_to_continue=False,
                speed_limit_ratio=ratio * ratio,
                phase_label="ШАР → ЗОГС",
                reason="8.9б / МНС 6.4.2 / 8.18: шар дохио — зогсох шугамын өмнө зогсоно",
            )
        else:
            # 8.19 — огцом тоормослох боломжгүй → аюулгүй үргэлжлүүлнэ
            return SignalDecision(
                can_proceed=True, must_stop=False, safe_to_continue=True,
                speed_limit_ratio=0.80,
                phase_label="ШАР → 8.19 ҮРГЭЛЖЛҮҮЛ",
                reason="ЗХД 8.19: шар дохио — огцом зогсох боломжгүй, аюулгүй үргэлжлүүлнэ",
            )

    # ── АНИВЧСАН ШАР (8.9в) — зохицуулгагүй уулзвар ─────────────────────
    if phase == SignalPhase.FLASHING_YEL.value:
        return SignalDecision(
            can_proceed=True, must_stop=False, safe_to_continue=True,
            speed_limit_ratio=0.55,
            phase_label="ШАР_АНИВЧ",
            reason="8.9в: анивчсан шар — зохицуулгагүй уулзвар, болгоомжтой зорчино",
        )

    # ── УЛААН ГЭРЭЛ (8.9г) болон ALL_RED ─────────────────────────────────
    brake_ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 58.0))

    # Нэмэлт дохиогоор явж болох эсэх шалгах (МНС 6.3.7)
    if add_sigs and evaluate_additional_signal_for_vehicle(
        ctx.turn, add_sigs, phase
    ):
        return SignalDecision(
            can_proceed=True, must_stop=False, safe_to_continue=True,
            speed_limit_ratio=0.6,
            phase_label="УЛААН + НЭМЭЛТ (6.3.7)",
            reason="МНС 6.3.7: улаан дохио ч гэсэн нэмэлт дохиогоор тухайн чиглэлд явна",
        )

    phase_label = (
        "БҮГД_УЛААН" if phase == SignalPhase.ALL_RED.value else "УЛААН"
    )
    return SignalDecision(
        can_proceed=False, must_stop=True, safe_to_continue=False,
        speed_limit_ratio=brake_ratio * brake_ratio,
        phase_label=phase_label,
        reason=f"8.9г / МНС 6.4.2 / 8.18: {phase_label} — зогсох шугамын өмнө заавал зогсоно",
    )


# ─────────────────────────────────────────────────────────────────────────────
# 3. ЗАМ ТАВЬЖ ӨГӨХ — ЗХД 10.2, 10.9, 15.8, 15.9, 16.1
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class YieldDecision:
    must_yield:  bool
    speed_ratio: float
    reason:      str


def evaluate_yield(
    ctx: TrafficRuleContext,
    stop_distance_px: float,
    approaching_from_right:  bool = False,
    is_main_road:            bool = True,
    pedestrian_crossing:     bool = False,
    pedestrian_in_crossing:  bool = False,
) -> YieldDecision:
    """ЗХД 10.2, 10.9, 15.8, 15.9, 16.1, 4.4 дагуу зам тавьж өгөх шаардлага."""

    if ctx.is_emergency:
        return YieldDecision(must_yield=False, speed_ratio=1.0,
                             reason="4.4: онцгой тусгай дохиотой — давуу эрхтэй")

    # 4.4 — орчинд онцгой тээврийн хэрэгсэл
    for other in ctx.same_lane_vehicles + ctx.opposite_vehicles:
        if other.get("type") == "emergency":
            ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 80.0))
            return YieldDecision(must_yield=True, speed_ratio=ratio * ratio,
                                 reason="4.4: онцгой тусгай дохиотой тээврийн хэрэгсэлд зам тавьж өгнө")

    # 16.1 — явган зорчигчийн зохицуулдаггүй гарц
    if pedestrian_crossing and pedestrian_in_crossing:
        ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 70.0))
        return YieldDecision(must_yield=True, speed_ratio=ratio * ratio,
                             reason="16.1: явган зорчигчийн гарцад зогсож зам тавьж өгнө")

    # 15.9 — туслах замаас гол зам руу
    if not is_main_road:
        ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 60.0))
        return YieldDecision(must_yield=True, speed_ratio=ratio * ratio,
                             reason="15.9: туслах замаас гол замын тээврийн хэрэгсэлд зам тавьж өгнө")

    # 15.8 — баруун гараас ирсэнд зам тавих
    if approaching_from_right:
        ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 55.0))
        return YieldDecision(must_yield=True, speed_ratio=ratio * ratio,
                             reason="15.8: баруун гараас ирсэн тээврийн хэрэгсэлд зам тавьж өгнө")

    # 10.9 — зүүн тийш эргэхдээ өөдөөс ирсэнд зам тавих
    if ctx.turn == "left":
        for other in ctx.opposite_vehicles:
            if other.get("turn", "straight") in ("straight", "right"):
                ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 60.0))
                return YieldDecision(must_yield=True, speed_ratio=ratio * ratio,
                                     reason="10.9: зүүн тийш эргэхдээ өөдөөс чигээрээ яваа тээврийн хэрэгсэлд зам тавьж өгнө")

    return YieldDecision(must_yield=False, speed_ratio=1.0,
                         reason="зам тавьж өгөх шаардлагагүй")


# ─────────────────────────────────────────────────────────────────────────────
# 4. ГҮЙЦЭЖ ТҮРҮҮЛЭХ — ЗХД 13.2, 13.4
# ─────────────────────────────────────────────────────────────────────────────

def can_overtake(
    ctx: TrafficRuleContext,
    at_intersection:             bool = False,
    at_pedestrian_crossing:      bool = False,
    in_school_zone:              bool = False,
    at_railway:                  bool = False,
    on_bridge:                   bool = False,
    in_tunnel:                   bool = False,
    limited_visibility:          bool = False,
    vehicle_ahead_is_overtaking: bool = False,
    vehicle_behind_is_overtaking:bool = False,
    front_vehicle_turning_left:  bool = False,
    oncoming_danger:             bool = False,
) -> tuple[bool, str]:
    if oncoming_danger:
        return False, "13.2а: өөдөөс яваа тээврийн хэрэгсэлд аюул учруулахаар байвал хориглоно"
    if vehicle_ahead_is_overtaking:
        return False, "13.2б: урд яваа тээврийн хэрэгсэл гүйцэж түрүүлэх үйлдэл хийж байна"
    if front_vehicle_turning_left:
        return False, "13.2в: урд яваа тээврийн хэрэгсэл зүүн гар тийш дохио өгч байна"
    if vehicle_behind_is_overtaking:
        return False, "13.2г: араас яваа тээврийн хэрэгсэл гүйцэж түрүүлэхээр эхэлсэн"
    if at_intersection:
        return False, "13.4а: уулзвар болон гарц дээр хориглоно"
    if at_pedestrian_crossing:
        return False, "13.4б: явган хүний гарц дээр хориглоно"
    if in_school_zone:
        return False, "13.4в: хороолол болон сургуулийн орчимд хориглоно"
    if at_railway:
        return False, "13.4г: төмөр замын гарам дээр болон 100 м-ийн дотор хориглоно"
    if on_bridge or in_tunnel:
        return False, "13.4д/е: гүүрэн дээр болон хонгил дотор хориглоно"
    if limited_visibility:
        return False, "13.4ж: үзэгдэлт хязгаарлагдмал хэсэгт хориглоно"
    return True, "гүйцэж түрүүлэхийг зөвшөөрнө"


# ─────────────────────────────────────────────────────────────────────────────
# 5. АЮУЛГҮЙ ЗАЙ — ЗХД 11.14
# ─────────────────────────────────────────────────────────────────────────────

def required_following_distance_px(
    speed_px_sec: float,
    vehicle_length_px: float = 20.0,
) -> float:
    """11.14 — хурдаас хамааран аюулгүй дагах зайг pixel-ээр тооцно."""
    reaction_dist = speed_px_sec * 0.8
    braking_dist  = (speed_px_sec ** 2) / (2.0 * 70.0)
    safety_buffer = vehicle_length_px * 1.5
    return reaction_dist + braking_dist + safety_buffer


def get_following_speed_ratio(
    gap_to_leader_px: float,
    speed_px_sec: float,
    vehicle_length_px: float = 20.0,
) -> float:
    """11.14 — урдаа яваа тээврийн хэрэгслээс аюулгүй зайг барих хурдны харьцаа."""
    required  = required_following_distance_px(speed_px_sec, vehicle_length_px)
    hard_stop = vehicle_length_px * 0.9
    if gap_to_leader_px <= hard_stop:
        return 0.0
    if gap_to_leader_px < required:
        ratio = (gap_to_leader_px - hard_stop) / max(1.0, required - hard_stop)
        return round(max(0.0, min(1.0, ratio)), 3)
    return 1.0


# ─────────────────────────────────────────────────────────────────────────────
# 6. ОНЦГОЙ ТУСГАЙ ДОХИОТОЙ ТЭЭВРИЙН ХЭРЭГСЭЛ — ЗХД 4.2, 4.4
# ─────────────────────────────────────────────────────────────────────────────

def handle_emergency_vehicle(
    ctx: TrafficRuleContext,
    emergency_direction: str,
    stop_distance_px: float,
) -> dict[str, Any]:
    """4.4 — онцгой тусгай дохиотой тээврийн хэрэгсэлд зам тавьж өгнө."""
    if ctx.is_emergency:
        return {"should_pull_over": False, "speed_ratio": 1.0,
                "reason": "4.2: өөрөө онцгой тусгай дохиотой — давуу эрхтэй"}
    ratio = max(0.0, min(1.0, (stop_distance_px - 4.0) / 80.0))
    return {"should_pull_over": True, "speed_ratio": ratio * ratio,
            "reason": "4.4: тусгай дуут болон гэрлэн дохио ажиллуулсан тээврийн хэрэгсэлд зам тавьж өгнө"}


# ─────────────────────────────────────────────────────────────────────────────
# 7. НЭГТГЭСЭН ДҮРМИЙН ФУНКЦ — _move_vehicle дотор дуудагдана
# ─────────────────────────────────────────────────────────────────────────────

def apply_traffic_rules(
    vehicle: dict[str, Any],
    all_vehicles: list[dict[str, Any]],
    signal_state: str,
    active_dir: str,
    stop_distance_px: float,
    dynamics: dict[str, float],
    zone_type: ZoneType = ZoneType.RESIDENTIAL,
    at_intersection:        bool = False,
    at_pedestrian_crossing: bool = False,
    pedestrian_in_crossing: bool = False,
    in_school_zone:         bool = False,
    emergency_directions:   list[str] | None = None,
    is_carrying_children:   bool = False,
    is_towing:              bool = False,
    approaching_from_right: bool = False,
    is_main_road:           bool = True,
    additional_signals:     list[AdditionalSignalMNS] | None = None,
    is_low_volume:          bool = False,
) -> dict[str, Any]:
    """
    МНС 4596:2007 болон ЗХД-ийн бүх дүрмийг нэгтгэн
    target_speed болон waiting төлвийг буцаана.
    """
    direction    = vehicle["dir"]
    vehicle_type = vehicle.get("type", "car")
    x            = float(vehicle["x"])
    y            = float(vehicle["y"])
    speed        = float(vehicle.get("speed", 0.0))
    turn         = vehicle.get("turn", "straight")
    turn_prog    = float(vehicle.get("turnProgress", 0.0))
    is_emergency = vehicle_type == "emergency"
    emerg_dirs   = set(emergency_directions or [])

    has_nearby_emergency = any(
        v.get("type") == "emergency"
        for v in all_vehicles if v["id"] != vehicle["id"]
    )

    same_lane = [
        v for v in all_vehicles
        if v["id"] != vehicle["id"]
        and v["dir"] == direction
        and v.get("lane") == vehicle.get("lane")
    ]
    opp_dir = {"north": "south", "south": "north", "east": "west", "west": "east"}
    opposite = [
        v for v in all_vehicles
        if v["id"] != vehicle["id"] and v["dir"] == opp_dir.get(direction, "")
    ]

    ctx = TrafficRuleContext(
        vehicle_id=vehicle["id"],
        direction=direction, vehicle_type=vehicle_type,
        x=x, y=y, speed=speed, lane=vehicle.get("lane", 0),
        turn=turn, turn_progress=turn_prog,
        signal_phase=signal_state, active_dir=active_dir,
        zone_type=zone_type,
        is_carrying_children=is_carrying_children,
        is_towing=is_towing, is_emergency=is_emergency,
        same_lane_vehicles=same_lane, opposite_vehicles=opposite,
    )

    applied_rules: list[str] = []
    target_speed  = dynamics["cruise"]
    waiting       = False
    must_stop     = False
    yield_reason  = ""
    signal_reason = ""

    # A. 4.4 — ойрын онцгой тээврийн хэрэгсэлд зам тавих
    if has_nearby_emergency and not is_emergency:
        emerg_action = handle_emergency_vehicle(ctx, direction, stop_distance_px)
        if emerg_action["should_pull_over"]:
            target_speed = min(target_speed, dynamics["cruise"] * emerg_action["speed_ratio"])
            waiting = emerg_action["speed_ratio"] < 0.1
            applied_rules.append(emerg_action["reason"])

    # B. 12.4/12.5 — хурдны хязгаар
    speed_limit = get_speed_limit_px_sec(ctx)
    if target_speed > speed_limit:
        target_speed = speed_limit
        applied_rules.append(f"12.4/12.5: хурдны хязгаар {speed_limit:.1f} px/sec")

    # C. МНС 6.4.2 + ЗХД 8.9/8.18/8.19 — гэрлэн дохионы логик
    phase_dirs = (
        ("north", "south") if active_dir in ("north", "south") else ("east", "west")
    )
    direction_has_green = direction in phase_dirs

    if not direction_has_green:
        signal_dec = evaluate_signal(
            ctx, stop_distance_px, speed, dynamics["brake"],
            additional_signals=additional_signals,
            is_low_volume=is_low_volume,
        )
        signal_reason = signal_dec.reason
        if signal_dec.must_stop:
            target_speed = min(target_speed, dynamics["cruise"] * signal_dec.speed_limit_ratio)
            must_stop = True
            waiting   = signal_dec.speed_limit_ratio < 0.05
            applied_rules.append(f"[{signal_dec.phase_label}] {signal_reason}")
        elif not signal_dec.can_proceed:
            target_speed = min(target_speed, dynamics["cruise"] * signal_dec.speed_limit_ratio)
            applied_rules.append(f"[{signal_dec.phase_label}] {signal_reason}")

    # D. 10.9/15.8/15.9/16.1 — зам тавьж өгөх
    yield_dec = evaluate_yield(
        ctx, stop_distance_px,
        approaching_from_right=approaching_from_right,
        is_main_road=is_main_road,
        pedestrian_crossing=at_pedestrian_crossing,
        pedestrian_in_crossing=pedestrian_in_crossing,
    )
    yield_reason = yield_dec.reason
    if yield_dec.must_yield:
        target_speed = min(target_speed, dynamics["cruise"] * yield_dec.speed_ratio)
        waiting      = yield_dec.speed_ratio < 0.05
        applied_rules.append(yield_reason)

    # E. 11.14 — аюулгүй дагах зай
    if same_lane:
        min_gap = float("inf")
        for other in same_lane:
            if other.get("turnProgress", 0.0) > 0.05:
                continue
            ox, oy = float(other["x"]), float(other["y"])
            if direction == "north":   gap = oy - y
            elif direction == "south": gap = y - oy
            elif direction == "east":  gap = ox - x
            else:                      gap = x - ox
            if 0 < gap < 200.0:
                min_gap = min(min_gap, gap)
        if min_gap < float("inf"):
            follow_ratio = get_following_speed_ratio(min_gap, speed)
            target_speed = min(target_speed, dynamics["cruise"] * follow_ratio)
            if follow_ratio < 1.0:
                applied_rules.append(
                    f"11.14: аюулгүй зай gap={min_gap:.1f}px ratio={follow_ratio:.2f}"
                )
            if follow_ratio < 0.05:
                waiting = True

    # F. 13.2/13.4 — гүйцэж түрүүлэх
    overtake_ok, overtake_reason = can_overtake(
        ctx, at_intersection=at_intersection,
        at_pedestrian_crossing=at_pedestrian_crossing,
        in_school_zone=in_school_zone,
    )

    target_speed = max(0.0, min(target_speed, dynamics["cruise"]))

    return {
        "target_speed":  target_speed,
        "waiting":       waiting,
        "must_stop":     must_stop,
        "yield_reason":  yield_reason,
        "signal_reason": signal_reason,
        "overtake_ok":   overtake_ok,
        "applied_rules": applied_rules,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 8. СИМУЛЯТОРТОЙ УЯЛДАХ ТУСЛАХ ФУНКЦҮҮД
# ─────────────────────────────────────────────────────────────────────────────

def build_mns_signal_timing_for_simulator(
    ai_green_times: dict[str, int],
) -> dict[str, SignalTimingMNS]:
    """
    AI тооцоосон ногоон гэрлийн хугацаануудаас
    МНС 6.4.2-ийн бүрэн горимын хугацааны бүтцийг үүсгэнэ.
    Симуляторын _refresh_green_times()-д дуудна.
    """
    return {
        direction: mns_signal_timing(green_sec)
        for direction, green_sec in ai_green_times.items()
    }


def get_simulator_phase_duration(
    current_phase: str,
    green_sec: int,
) -> int:
    """
    Симуляторын phase_timer-д тавих хугацааг МНС 6.4.2 дагуу буцаана.
    _tick() дотор phase солих үед дуудна.
    """
    return {
        SignalPhase.GREEN.value:       max(12, green_sec),
        SignalPhase.GREEN_FLASH.value: MNS_GREEN_FLASH_SEC,
        SignalPhase.YELLOW.value:      MNS_YELLOW_SEC,
        SignalPhase.ALL_RED.value:     MNS_ALL_RED_SEC,
        SignalPhase.YELLOW_RED.value:  MNS_YELLOW_RED_SEC,
        SignalPhase.RED.value:         0,
    }.get(current_phase, MNS_YELLOW_SEC)


def get_zone_from_sim_mode(
    mode: str,
    in_school_zone: bool = False,
    in_district:    bool = False,
) -> ZoneType:
    if in_school_zone or in_district:
        return ZoneType.SCHOOL_ZONE
    if mode == "highway":
        return ZoneType.HIGHWAY
    if mode == "rural":
        return ZoneType.RURAL
    return ZoneType.RESIDENTIAL


def format_applied_rules(applied_rules: list[str]) -> str:
    if not applied_rules:
        return "дүрмийн хязгаарлалтгүй"
    return " | ".join(applied_rules)


def log_rule_violation(vehicle_id: int, rule: str, details: str) -> dict[str, Any]:
    return {"vehicle_id": vehicle_id, "rule": rule, "details": details}