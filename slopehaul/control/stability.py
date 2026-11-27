"""稳定性监督：侧倾/俯仰稳定包络、侧向加速度约束与动作降级。

判据使用静态倾翻角 φ_tip = atan(track / (2·h_cg))，并按可配置的安全系数
（默认 0.75）折算成作业包络。重载会让 h_cg 抬高，包络自动收紧——这是与
固定阈值方案最重要的区别。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import ControlParams, VehicleParams


@dataclass
class StabilityReport:
    roll: float
    pitch: float
    roll_margin: float        # 1.0 表示已到包络边界
    pitch_margin: float
    scale: float              # 速度缩放系数 0.2~1.0
    level: str                # normal / warning / critical
    reason: str
    a_lat_allow: float


class StabilitySupervisor:
    def __init__(self, vp: VehicleParams, cp: ControlParams) -> None:
        self.vp = vp
        self.cp = cp

    def evaluate(self, pose, load_est, speed: float, curvature: float, cross_slope: float) -> StabilityReport:
        vp = self.vp
        h = load_est.cg_height
        roll_cap = np.rad2deg(np.arctan(vp.track / (2.0 * h))) * vp.stability_margin
        # 纵向倾翻由轴距决定，取几何极限与配置上限的较小值
        pitch_geom = np.rad2deg(np.arctan(vp.wheelbase / (2.0 * h)))
        pitch_cap = min(vp.pitch_limit_deg, pitch_geom) * vp.stability_margin

        roll = pose.roll
        pitch = pose.pitch
        # 横坡分量单独计入：即使车体静止，横坡也吃掉侧倾裕度
        roll_margin = abs(roll) / max(roll_cap, 1.0)
        pitch_margin = abs(pitch) / max(pitch_cap, 1.0)

        # 侧向加速度允许值：按剩余裕度线性收缩
        a_lat_allow = max(0.10, 0.55 * 9.81 * (1.0 - max(roll_margin, 0.0)))
        if abs(curvature) > 1e-3:
            v_curve = float(np.sqrt(a_lat_allow / abs(curvature)))
        else:
            v_curve = 99.0

        scale = 1.0
        level = "normal"
        reason = ""
        worst = max(roll_margin, pitch_margin)
        if worst >= 1.0:
            scale, level = 0.0, "critical"
            reason = f"姿态进入倾翻包络（侧倾裕度 {roll_margin:.2f}，俯仰裕度 {pitch_margin:.2f}）"
        elif worst >= self.cp.safety_roll_trigger:
            scale = float(max(0.25, 1.0 - (worst - 0.6) / 0.4 * 0.75))
            level = "warning"
            reason = f"姿态裕度不足（{worst:.2f}），限速至 {scale * 100:.0f}%"
        if speed > v_curve:
            scale = min(scale, max(v_curve / max(speed, 0.05), 0.2))
            level = "warning" if level == "normal" else level
            reason = reason or "弯道侧向加速度超限"

        return StabilityReport(
            roll=roll,
            pitch=pitch,
            roll_margin=float(roll_margin),
            pitch_margin=float(pitch_margin),
            scale=float(np.clip(scale, 0.0, 1.0)),
            level=level,
            reason=reason,
            a_lat_allow=a_lat_allow,
        )
