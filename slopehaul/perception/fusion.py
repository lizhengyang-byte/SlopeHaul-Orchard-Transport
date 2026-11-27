"""姿态融合：IMU 陀螺高频 + 高程图几何低频，构成互补滤波。

坡地工况下 IMU 会因振动与安装偏置漂移，高程图则受噪声与遮挡影响且更新频率低；
两者互补后给出可用于稳定包络判断的俯仰/侧倾角。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PoseEstimate:
    pitch: float
    roll: float
    pitch_rate: float
    roll_rate: float
    source: str


class ComplementaryPose:
    def __init__(self, alpha: float = 0.82) -> None:
        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0
        self.initialized = False

    def update(
        self,
        dt: float,
        pitch_imu: float,
        roll_imu: float,
        pitch_rate: float,
        yaw_rate: float,
        perception_grade: float,
        perception_cross: float,
        perception_trust: float,
    ) -> PoseEstimate:
        if not self.initialized:
            self.pitch = pitch_imu
            self.roll = roll_imu
            self.initialized = True

        # 陀螺积分（用横摆角速度近似侧倾速率的上界）
        self.pitch = self.alpha * (self.pitch + pitch_rate * dt) + (1 - self.alpha) * pitch_imu
        self.roll = self.alpha * (self.roll + yaw_rate * 20.0 * dt) + (1 - self.alpha) * roll_imu

        # 高程图修正：仅在几何量可信时介入
        if perception_trust > 0.5:
            w = 0.25 * perception_trust
            self.pitch = (1 - w) * self.pitch + w * perception_grade
            self.roll = (1 - w) * self.roll + w * perception_cross

        self.pitch = float(np.clip(self.pitch, -50, 50))
        self.roll = float(np.clip(self.roll, -50, 50))
        return PoseEstimate(
            pitch=self.pitch,
            roll=self.roll,
            pitch_rate=pitch_rate,
            roll_rate=0.0,
            source="imu+elevation",
        )
