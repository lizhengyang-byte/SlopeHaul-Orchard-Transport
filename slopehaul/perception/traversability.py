"""可通行性评估：把几何特征 + 土壤分类 + 在线附着估计合成为通行代价与限速。

输出三个量给控制器：
1. `blocked` —— 是否绝对不可通行（超坡度/台阶/粗糙度上限）；
2. `v_limit` —— 由“制动距离 ≤ 可视距离”和“侧向附着”共同决定的限速；
3. `risk` —— 0~1 的风险分，用于速度规划中的连续降速，而不是硬开关。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import ControlParams, PerceptionParams
from .elevation import TerrainFeatures
from .soil_classifier import SoilReport


@dataclass
class TraversabilityResult:
    features: TerrainFeatures
    soil: SoilReport
    cost: float
    risk: float
    blocked: bool
    block_reason: str
    v_limit: float
    mu_used: float
    stopping_dist: float
    slope_limit: float


def evaluate(
    feats: TerrainFeatures,
    soil: SoilReport,
    pp: PerceptionParams,
    cp: ControlParams,
    mu_online: float | None,
    speed: float,
    load: float,
    curvature: float,
    lookahead: float,
) -> TraversabilityResult:
    grade = feats.grade_deg
    slope = max(abs(feats.slope_deg), abs(grade), abs(feats.cross_deg))
    # 土壤先验与在线辨识融合：先验给冷启动，在线估计给真实附着
    # 视觉土壤分类给粗先验，触地滑移辨识只在 ±0.08 范围内修正：
    # 避免单帧误分类或估计器瞬态直接推翻通行判定（竞赛现场最怕“假阴性”）。
    if mu_online is None:
        mu = soil.mu_prior
    else:
        span = 0.05 if soil.proba.max() >= 0.75 else 0.08
        mu = soil.mu_prior + float(np.clip(mu_online - soil.mu_prior, -span, span))
    mu = float(np.clip(mu, 0.20, 0.90))
    crr = soil.crr_prior

    mu_slope = float(np.rad2deg(np.arctan(0.85 * mu)))
    slope_limit = min(soil.slope_limit, pp.slope_block_deg, mu_slope)
    blocked = False
    reason = ""
    if slope > slope_limit + 1.0:
        blocked, reason = True, f"坡度 {slope:.1f}° 超过 {soil.soil_name} 通行上限 {slope_limit:.0f}°"
    elif feats.roughness > pp.roughness_block + 0.005:
        blocked, reason = True, f"粗糙度 {feats.roughness * 100:.1f} cm 超过 {pp.roughness_block * 100:.0f} cm"
    elif feats.step_height > pp.step_block:
        blocked, reason = True, f"台阶 {feats.step_height * 100:.1f} cm 超过 {pp.step_block * 100:.0f} cm"

    # 纵向可用减速度（上坡时变小，下坡时变大）
    theta = np.deg2rad(grade)
    a_dec = mu * 9.81 * np.cos(theta) - 9.81 * np.sin(theta) - crr * 9.81
    a_dec = max(a_dec, 0.0)

    # 制动距离约束：预留 35% 余量
    if a_dec > 0.05:
        v_brake = float(np.sqrt(max(2.0 * a_dec * lookahead * cp.stop_decel_reserve, 0.0)))
    else:
        v_brake = 0.0

    # 侧向约束：转弯时不允许把附着全部用于纵向
    a_lat_max = max(0.45 * mu * 9.81, 0.4)
    if abs(curvature) > 1e-3:
        v_curve = float(np.sqrt(a_lat_max / abs(curvature)))
    else:
        v_curve = cp.max_speed_flat

    # 重载惩罚：载重越大，制动距离与侧倾风险越高
    load_penalty = 1.0 - 0.22 * min(load / 200.0, 1.0)

    v_limit = max(min(v_brake, v_curve) * load_penalty, 0.0)
    v_limit = min(v_limit, cp.max_speed_flat)

    cost = (
        0.45 * (slope / max(slope_limit, 1.0)) ** 2
        + 0.25 * min(feats.roughness / pp.roughness_block, 1.5)
        + 0.15 * min(feats.step_height / pp.step_block, 1.5)
        + 0.15 * (1.0 - mu / 0.85)
    )
    risk = float(np.clip(cost, 0.0, 1.0))
    stopping = speed ** 2 / (2.0 * a_dec) if a_dec > 0.05 else float("inf")

    return TraversabilityResult(
        features=feats,
        soil=soil,
        cost=float(cost),
        risk=risk,
        blocked=blocked,
        block_reason=reason,
        v_limit=float(v_limit),
        mu_used=mu,
        stopping_dist=float(stopping),
        slope_limit=float(slope_limit),
    )
