"""失效安全（failsafe）状态机，对齐 ArduRover 的失控保护分层。

OK → DERATE → HOLD → STOP 四级；急停锁存，需操作员显式复位。
与 control/safety.py 分工：safety 逐指令裁剪，本模块负责任务级降级与停机。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


class Level(IntEnum):
    OK = 0
    DERATE = 1
    HOLD = 2
    STOP = 3

    @property
    def label(self) -> str:
        return {0: "OK", 1: "DERATE", 2: "HOLD", 3: "STOP"}[int(self)]


@dataclass
class Action:
    level: Level
    reasons: tuple[str, ...] = ()
    latched: bool = False
    speed_scale: float = 1.0

    def summary(self) -> dict:
        return {
            "level": str(self.level.label),
            "reasons": list(self.reasons),
            "latched": self.latched,
            "speed_scale": round(self.speed_scale, 3),
        }


@dataclass
class FailsafeConfig:
    heartbeat_timeout_s: float = 0.6
    s_range: tuple[float, float] = (0.0, 1e9)
    y_limit_m: float = 0.9
    tilt_max_deg: float = 32.0
    derate_temp_c: float = 75.0
    stop_temp_c: float = 85.0


class FailsafeManager:
    def __init__(self, cfg: FailsafeConfig | None = None) -> None:
        self.cfg = cfg or FailsafeConfig()
        self._estop_latched = False
        self._history: list[dict] = []

    def clear_estop(self, operator_ack: bool = True) -> bool:
        """急停只能由操作员显式复位；未确认时保持锁存。"""

        if not operator_ack:
            return False
        self._estop_latched = False
        return True

    def step(self, t: float, link_age_s: float, s: float, y: float, roll_deg: float, pitch_deg: float, motor_temp_c: float, estop: bool = False) -> Action:
        if estop:
            self._estop_latched = True

        stop_reasons: list[str] = []
        hold_reasons: list[str] = []
        derate_reasons: list[str] = []

        if self._estop_latched:
            stop_reasons.append("外部急停（已锁存，需操作员复位）")
        tilt = max(abs(roll_deg), abs(pitch_deg))
        if tilt > self.cfg.tilt_max_deg:
            stop_reasons.append(f"姿态 {tilt:.1f}° 超过 {self.cfg.tilt_max_deg:.0f}°")
        if motor_temp_c >= self.cfg.stop_temp_c:
            stop_reasons.append(f"电机 {motor_temp_c:.0f}℃ 过温停机")

        if link_age_s > self.cfg.heartbeat_timeout_s:
            hold_reasons.append(f"链路心跳中断 {link_age_s:.2f}s")
        if not (self.cfg.s_range[0] <= s <= self.cfg.s_range[1]) or abs(y) > self.cfg.y_limit_m:
            hold_reasons.append(f"越出地理围栏 s={s:.2f}, y={y:.2f}")

        if self.cfg.derate_temp_c <= motor_temp_c < self.cfg.stop_temp_c:
            derate_reasons.append(f"电机 {motor_temp_c:.0f}℃ 降载运行")

        if stop_reasons:
            action = Action(Level.STOP, tuple(stop_reasons), latched=self._estop_latched, speed_scale=0.0)
        elif hold_reasons:
            action = Action(Level.HOLD, tuple(hold_reasons), speed_scale=0.0)
        elif derate_reasons:
            action = Action(Level.DERATE, tuple(derate_reasons), speed_scale=0.7)
        else:
            action = Action(Level.OK, (), speed_scale=1.0)

        self._history.append({"t": round(float(t), 2), **action.summary()})
        return action

    @property
    def history(self) -> list[dict]:
        return list(self._history)
