"""坡道驻车与起步状态机。

状态：DRIVE → BRAKING → HOLD → RELEASE → DRIVE。
关键判断不是“拉手刹”，而是“静摩擦是否真的够”：把实测/估计附着换算成可提供的
驻车力，与坡道下滑分力比较；不够就提前告警并请求降级（减载、反向找平、绕行）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..config import ControlParams, VehicleParams


@dataclass
class HoldReport:
    state: str
    brake: float
    hold_capacity: float
    required_force: float
    margin: float
    slipping: bool
    message: str = ""


class HillHoldController:
    def __init__(self, vp: VehicleParams, cp: ControlParams) -> None:
        self.vp = vp
        self.cp = cp
        self.state = "DRIVE"
        self.brake = 0.0
        self.release_timer = 0.0
        self.slip_distance = 0.0
        self._prev_s = None

    def reset(self) -> None:
        self.state = "DRIVE"
        self.brake = 0.0
        self.release_timer = 0.0
        self.slip_distance = 0.0

    def update(
        self,
        dt: float,
        s: float,
        speed: float,
        grade: float,
        mu_est: float,
        normal_sum: float,
        mass: float,
        crr: float,
        stop_request: bool,
    ) -> HoldReport:
        theta = np.deg2rad(grade)
        need = max(mass * 9.81 * np.sin(theta) - crr * mass * 9.81 * np.cos(theta), 0.0)
        cap_brake = self.vp.brake_torque * self.vp.n_wheels / self.vp.wheel_radius
        cap_friction = self.vp.static_gain * mu_est * normal_sum
        cap = min(cap_brake, cap_friction)
        margin = (cap - need) / max(need, 1.0) if need > 0 else 1.0

        slipping = False
        msg = ""

        if self.state == "DRIVE":
            if stop_request:
                self.state = "BRAKING"
        elif self.state == "BRAKING":
            self.brake = min(1.0, self.brake + dt / 0.35)
            if abs(speed) < 0.06:
                self.state = "HOLD"
        elif self.state == "HOLD":
            self.brake = self.cp.hill_hold_brake_ratio
            if margin < 0.0:
                # 静态能力不足：制动拉满，记录溜坡
                self.brake = 1.0
                slipping = True
                msg = "驻车力不足，车辆后溜，请减载或改走缓坡"
            if not stop_request:
                self.state = "RELEASE"
                self.release_timer = 0.0
        elif self.state == "RELEASE":
            self.release_timer += dt
            self.brake = max(0.0, 1.0 - self.release_timer / 0.45)
            if self.brake <= 0.01:
                self.state = "DRIVE"
                self.brake = 0.0

        if self._prev_s is not None and slipping:
            self.slip_distance += max(self._prev_s - s, 0.0)
        self._prev_s = s

        return HoldReport(
            state=self.state,
            brake=float(self.brake),
            hold_capacity=float(cap),
            required_force=float(need),
            margin=float(margin),
            slipping=slipping,
            message=msg,
        )
