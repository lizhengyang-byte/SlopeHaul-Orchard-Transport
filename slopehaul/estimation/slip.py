"""速度 / 滑移率 / 附着系数在线估计。

对应 ROSE（Robust Off-road wheel odometry with Slip Estimation）的思路：
轮速在打滑时不可信，因此用 IMU 加速度做预测、用“低滑移时才采信轮速”的自适应
门限做更新，得到打滑工况下仍然可用的纵向速度；再据此反算各轮滑移率，
并用递推最小二乘（遗忘因子）在线辨识峰值附着系数。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import VehicleParams


@dataclass
class SpeedEstimate:
    v: float
    v_wheel: float
    slip: np.ndarray
    confidence: float
    slip_ratio_mean: float


class VelocityEKF:
    """两状态 EKF：纵向速度 v 与加速度零偏 b。"""

    def __init__(self, vp: VehicleParams) -> None:
        self.vp = vp
        self.x = np.zeros(2)
        self.p = np.eye(2) * np.array([[0.04, 0.0], [0.0, 0.25]])
        self.q = np.diag([0.02, 0.008])
        self.initialized = False

    def reset(self, v: float = 0.0) -> None:
        self.x = np.array([v, 0.0])
        self.initialized = True

    def predict(self, accel_meas: float, dt: float) -> None:
        if not self.initialized:
            self.reset(0.0)
        # x = [v, b]; v' = v + (a - b)dt
        f = np.array([[1.0, -dt], [0.0, 1.0]])
        self.x = f @ self.x + np.array([accel_meas * dt, 0.0])
        self.p = f @ self.p @ f.T + self.q * dt
        self.x[0] = max(self.x[0], -2.0)

    def update_wheel(self, v_wheel: float, wheel_sigma: float, slip_proxy: float) -> None:
        # 滑移越大，轮速观测的等效噪声越大（自适应门限）
        r = wheel_sigma ** 2 + (0.35 * min(slip_proxy, 1.0)) ** 2
        h = np.array([[1.0, 0.0]])
        s = h @ self.p @ h.T + r
        k = (self.p @ h.T) / s
        innovation = v_wheel - self.x[0]
        self.x = self.x + (k.flatten() * innovation)
        self.p = (np.eye(2) - k @ h) @ self.p
        self.p = 0.5 * (self.p + self.p.T)

    @property
    def v(self) -> float:
        return float(self.x[0])


class FrictionRLS:
    """由“利用率 = μ_max·g(s)”反解 μ_max，递推最小二乘带遗忘因子。"""

    def __init__(self, slip_opt: float = 0.18, lam: float = 0.97, init: float = 0.55) -> None:
        self.slip_opt = slip_opt
        self.lam = lam
        self.theta = init
        self.p = 1.0

    def update(self, slip: float, mu_util: float, weight: float) -> None:
        ratio = max(abs(slip), 0.02) / self.slip_opt
        gain = (2.0 * ratio) / (1.0 + ratio ** 2)
        if gain < 0.40:
            return
        y = min(mu_util / gain, 1.0)
        # 标量递推最小二乘（回归量恒为 1）
        k = self.p / (self.lam + self.p)
        self.theta = float(self.theta + weight * k * np.clip(y - self.theta, -0.08, 0.08))
        self.p = float((1.0 - k) * self.p / self.lam + 1e-6)
        self.theta = float(np.clip(self.theta, 0.25, 0.88))

    @property
    def mu_max(self) -> float:
        return self.theta


class SlipEstimator:
    """组合轮速 + IMU + 视觉里程计，输出车速、滑移率与置信度。"""

    def __init__(self, vp: VehicleParams, mu_init: float = 0.55) -> None:
        self.vp = vp
        self.ekf = VelocityEKF(vp)
        self.rls = FrictionRLS(vp.slip_opt, init=mu_init)
        self.v = 0.0
        self.confidence = 0.0
        self.slip = np.zeros(vp.n_wheels)

    def update(
        self,
        dt: float,
        wheel_omega: np.ndarray,
        accel_meas: float,
        ground_speed_meas: float,
        normal_loads: np.ndarray,
        traction_meas: float,
        grade_deg: float,
        mass: float,
    ) -> SpeedEstimate:
        v_wheel = float(np.mean(wheel_omega)) * self.vp.wheel_radius
        slip_proxy = abs(v_wheel - self.v)

        self.ekf.predict(accel_meas, dt)
        # 视觉/卫星里程计作为弱观测
        self.ekf.update_wheel(ground_speed_meas, 0.06, slip_proxy)
        # 低滑移时轮速是强观测
        gate = max(0.0, 1.0 - slip_proxy / 0.35)
        if gate > 0.1:
            self.ekf.update_wheel(v_wheel, 0.03 / gate, slip_proxy)

        self.v = max(self.ekf.v, 0.0)
        self.confidence = float(np.clip(gate, 0.0, 1.0))

        # 各轮滑移率
        wv = wheel_omega * self.vp.wheel_radius
        denom = np.maximum(np.abs(wv), 0.18)
        self.slip = np.clip((wv - self.v) / denom, -0.5, 1.0)

        # 附着利用率 = 实际传递的牵引力 / 法向载荷（不需要再用坡度重复计入重力分量）
        load_sum = max(float(np.sum(normal_loads)), 1.0)
        mu_util = float(np.clip(traction_meas / load_sum, 0.0, 1.0))
        # 只在“有滑移但未失控”的窗口内辨识，滑移过低不可观、过高则已进入附着跌落段
        slip_mean = float(np.mean(np.abs(self.slip)))
        weight = 0.0
        if 0.08 <= slip_mean <= 0.45 and abs(accel_meas) < 2.0:
            weight = 0.35 if self.confidence < 0.4 else 0.8
        self.rls.update(slip_mean, mu_util, weight)

        return SpeedEstimate(
            v=self.v,
            v_wheel=v_wheel,
            slip=self.slip.copy(),
            confidence=self.confidence,
            slip_ratio_mean=float(np.mean(np.abs(self.slip))),
        )
