"""2.5D 通过性代价层（对齐 Nav2-Traversability-Layer / terrain_aware_navigation）。

在 (s, y) 平面栅格采样高程，输出坡度、横坡、粗糙度、坡度校正后的台阶、土壤先验与代价；
不可通行区做一格膨胀（不确定性膨胀），避免贴着危险边缘走。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from slopehaul.config import DEFAULT_CONFIG

SOIL_MU = (0.80, 0.66, 0.62, 0.45)
SOIL_LIMIT_DEG = (28.0, 25.0, 22.0, 15.0)


@dataclass
class CostMap:
    s_axis: np.ndarray
    y_axis: np.ndarray
    elevation: np.ndarray
    slope_deg: np.ndarray
    cross_deg: np.ndarray
    roughness: np.ndarray
    step_m: np.ndarray
    cost: np.ndarray
    blocked: np.ndarray
    mu_prior: np.ndarray

    @property
    def rows(self) -> int:
        return int(self.s_axis.size)

    @property
    def cols(self) -> int:
        return int(self.y_axis.size)

    def center_col(self) -> int:
        return int(np.argmin(np.abs(self.y_axis)))

    def stats(self) -> dict:
        return {
            "blocked_ratio": float(self.blocked.mean()),
            "mean_cost": float(self.cost.mean()),
            "max_step_m": float(self.step_m.max()),
            "max_slope_deg": float(self.slope_deg.max()),
            "max_cross_deg": float(self.cross_deg.max()),
            "max_roughness_m": float(self.roughness.max()),
        }


def _moving(a: np.ndarray, k: int, axis: int = 0) -> np.ndarray:
    """滑动平均（边缘用缩窗归一化）。"""

    kernel = np.ones(k)
    num = np.apply_along_axis(lambda c: np.convolve(c, kernel, mode="same"), axis, a)
    den = np.apply_along_axis(lambda c: np.convolve(np.ones_like(c), kernel, mode="same"), axis, a)
    return num / den


def build_costmap(terrain, s0: float = 0.0, s1: float | None = None, y_half: float = 0.9, res: float = 0.15, cfg=DEFAULT_CONFIG) -> CostMap:
    s1 = float(terrain.length) if s1 is None else float(s1)
    s_axis = np.arange(s0, s1 + 1e-9, res)
    y_axis = np.arange(-y_half, y_half + 1e-9, res)
    elev = np.array([[terrain.height(float(s), float(y)) for y in y_axis] for s in s_axis])

    dzds = np.gradient(elev, res, axis=0)
    slope = np.degrees(np.arctan(np.abs(dzds)))
    dzdy = np.gradient(elev, res, axis=1)
    cross = np.degrees(np.arctan(np.abs(dzdy)))

    trend = _moving(elev, 5, axis=0)
    resid = elev - trend
    roughness = np.sqrt(np.clip(_moving(resid * resid, 5, axis=0), 0.0, None))

    step_raw = np.zeros_like(elev)
    step_raw[:-1] = np.abs(np.diff(elev, axis=0))
    step_raw[-1] = step_raw[-2] if elev.shape[0] > 1 else 0.0
    # 坡度校正用平滑后的代表坡度：否则台阶自身的陡坡会把台阶抹掉
    ramp_slope = np.minimum(_moving(slope, 7, axis=0), 25.0)
    ramp = np.abs(np.tan(np.deg2rad(ramp_slope)) * res) * 1.2
    step_eff = np.clip(step_raw - ramp, 0.0, None)

    mu_prior = np.empty_like(elev)
    soil_lim = np.empty_like(elev)
    for i, s in enumerate(s_axis):
        cls = min(3, max(0, int(round(float(terrain.soil(float(s)))))))
        mu_prior[i, :] = SOIL_MU[cls]
        soil_lim[i, :] = SOIL_LIMIT_DEG[cls]

    vehicle = getattr(cfg, "vehicle", None)
    track = float(getattr(vehicle, "track", 1.10))
    h_cg = 0.60
    if vehicle is not None:
        raw = getattr(vehicle, "cg_height", 0.60)
        h_cg = float(raw) if np.isscalar(raw) else 0.60
    cross_cap = float(np.degrees(np.arctan(track / (2.0 * h_cg))) * 0.75)

    # 硬判据用“摩擦允许”：合成坡度超 atan(0.85μ) 才判死，土壤类别上限只进代价权重
    slope_limit = np.minimum(30.0, np.degrees(np.arctan(0.85 * mu_prior)))

    c_slope = np.clip((slope - 0.6 * slope_limit) / (0.4 * slope_limit), 0.0, 1.0)
    c_cross = np.clip((cross - 0.5 * cross_cap) / (0.5 * cross_cap), 0.0, 1.0)
    c_rough = np.clip((roughness - 0.04) / 0.06, 0.0, 1.0)
    c_step = np.clip((step_eff - 0.08) / 0.08, 0.0, 1.0)
    c_mu = np.clip((0.55 - mu_prior) / 0.15, 0.0, 1.0)
    cost = 0.32 * c_slope + 0.18 * c_cross + 0.18 * c_rough + 0.22 * c_step + 0.10 * c_mu

    blocked = (slope > slope_limit) | (cross > cross_cap) | (step_eff > 0.16) | (mu_prior < 0.48)

    def _dilate(mask: np.ndarray) -> np.ndarray:
        out = mask.copy()
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                shifted = np.zeros_like(mask)
                src = mask[max(0, di) : mask.shape[0] + min(0, di), max(0, dj) : mask.shape[1] + min(0, dj)]
                shifted[max(0, -di) : mask.shape[0] - max(0, di), max(0, -dj) : mask.shape[1] - max(0, dj)] = src
                out |= shifted
        return out

    blocked_d = _dilate(blocked)
    cost = np.clip(cost + 0.45 * blocked_d.astype(float), 0.0, 1.0)
    cost[blocked_d] = np.maximum(cost[blocked_d], 0.75)

    return CostMap(s_axis, y_axis, elev, slope, cross, roughness, step_eff, cost, blocked_d, mu_prior)
