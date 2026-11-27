"""安全监督层：越界、卡滞、过热、后溜的统一处置。

设计原则是“先降级、再停车、最后回退”，且每次介入都要记录原因，便于复盘与
竞赛现场取证；安全层独立于控制层，控制层给出的指令必须先过这一层。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import ControlParams


@dataclass
class SafetyDecision:
    allow: bool
    scale: float
    emergency_stop: bool
    reverse: bool
    code: str
    message: str


class SafetySupervisor:
    def __init__(self, cp: ControlParams) -> None:
        self.cp = cp
        self.stall_time = 0.0
        self.interventions = 0
        self.last_code = "OK"

    def reset(self) -> None:
        self.stall_time = 0.0
        self.interventions = 0
        self.last_code = "OK"

    def evaluate(
        self,
        dt: float,
        speed: float,
        speed_target: float,
        stability,
        torque_util: float,
        motor_temp: float,
        pitch: float,
        roll: float,
        blocked: bool,
        grade: float,
    ) -> SafetyDecision:
        # 1) 姿态硬限
        if abs(roll) > 45.0 or abs(pitch) > 45.0:
            self._log()
            return SafetyDecision(False, 0.0, True, False, "ATTITUDE_LIMIT", "姿态超出 45°，立即停车")
        # 2) 稳定包络耗尽
        if stability.level == "critical":
            self._log()
            return SafetyDecision(False, 0.0, True, False, "STABILITY_CRITICAL", stability.reason)
        # 3) 已判定不可通行却仍在前进：安全层直接否决
        if blocked and speed_target > 0.05:
            self._log()
            return SafetyDecision(False, 0.0, False, False, "BLOCKED_AHEAD", "前方不可通行，停车待决策")
        # 4) 卡滞：有扭矩需求但车速长期接近零
        if speed_target > 0.1 and speed < 0.05 and torque_util > 0.6:
            self.stall_time += dt
        else:
            self.stall_time = 0.0
        if self.stall_time > 3.0:
            self._log()
            return SafetyDecision(False, 0.0, False, True, "STALL_RECOVERY", "持续打滑卡滞，回退 0.5 m 后重试")
        # 5) 电机过热
        if motor_temp > 85.0:
            self._log()
            return SafetyDecision(True, 0.6, False, False, "MOTOR_HOT", "电机温度过高，降载 40%")

        if stability.level == "warning":
            return SafetyDecision(True, stability.scale, False, False, "STABILITY_WARNING", stability.reason)
        return SafetyDecision(True, 1.0, False, False, "OK", "")

    def _log(self) -> None:
        self.interventions += 1
