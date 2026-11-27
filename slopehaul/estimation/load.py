"""载重与重心估计。

坡地重载运输的关键变量之一是“货物重心变了没有”：同样坡度，满载时质心更高、
纵向更靠后，侧倾与后翻风险都会变化。这里用称重传感器 + 姿态做带坡度补偿的低通估计。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import VehicleParams


@dataclass
class LoadEstimate:
    payload: float
    mass: float
    cg_height: float
    cg_shift: float        # 相对空载质心的纵向偏移，m（正为靠后）
    confidence: float


class LoadEstimator:
    def __init__(self, vp: VehicleParams, tau: float = 0.6) -> None:
        self.vp = vp
        self.tau = tau
        self.payload = 0.0
        self.history: list[float] = []

    def update(self, dt: float, load_meas: float, grade_deg: float, accel_long: float) -> LoadEstimate:
        theta = np.deg2rad(grade_deg)
        # 坡度与加速度造成的称重误差补偿（传感器装在车架上的投影误差）
        bias = self.vp.mass(self.payload) * 9.81 * np.sin(theta) * 0.012 + accel_long * 0.8
        corrected = max(load_meas - bias, 0.0)
        alpha = min(dt / self.tau, 1.0)
        self.payload += (corrected - self.payload) * alpha
        self.payload = float(np.clip(self.payload, 0.0, self.vp.payload_max * 1.05))

        self.history.append(self.payload)
        if len(self.history) > 40:
            self.history.pop(0)
        std = float(np.std(self.history)) if len(self.history) > 5 else 5.0
        confidence = float(np.clip(1.0 - std / 15.0, 0.0, 1.0))

        ratio = self.payload / self.vp.payload_max
        cg_shift = 0.06 * ratio
        return LoadEstimate(
            payload=self.payload,
            mass=self.vp.mass(self.payload),
            cg_height=self.vp.cg_height(self.payload),
            cg_shift=cg_shift,
            confidence=confidence,
        )
