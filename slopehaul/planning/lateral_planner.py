"""沿运输路径的横向路径规划（联合重心投影约束），参考 AgRobPP。

在代价层上逐行选横向偏移（状态=栅格列，转移=相邻列），代价含通过性代价与曲率惩罚，
禁行栅格直接不可达。输出供速度规划器使用的横向偏移序列与可行性结论。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from slopehaul.perception.costmap import CostMap


@dataclass
class RoutePlan:
    s_axis: np.ndarray
    y_offset: np.ndarray
    feasible: bool
    cost_total: float
    center_cost_total: float
    blocked_on_center: int
    min_clearance_m: float
    max_cross_deg: float
    max_lateral_rate: float
    notes: list[str] = field(default_factory=list)

    @property
    def improvement(self) -> float:
        """相对“始终走中线”的代价降幅。"""

        if self.center_cost_total <= 1e-9:
            return 0.0
        return 1.0 - self.cost_total / self.center_cost_total

    def summary(self) -> dict:
        return {
            "feasible": self.feasible,
            "cost_total": round(self.cost_total, 3),
            "center_cost_total": round(self.center_cost_total, 3),
            "cost_improvement": round(self.improvement, 4),
            "blocked_on_center": int(self.blocked_on_center),
            "min_clearance_m": round(float(self.min_clearance_m), 3),
            "max_cross_deg": round(float(self.max_cross_deg), 2),
            "max_lateral_rate": round(float(self.max_lateral_rate), 3),
        }


def plan_route(cm: CostMap, y_limit: float = 0.5, curv_weight: float = 0.35, center_corridor: float = 0.25) -> RoutePlan:
    cols = cm.cols
    ys = cm.y_axis
    allowed = np.abs(ys) <= y_limit + 1e-9
    center_j = cm.center_col()

    if not allowed.any():
        raise ValueError("y_limit 过小，没有可用栅格")

    usable = allowed[None, :] & ~cm.blocked
    cost = np.where(usable, cm.cost, np.inf)
    dp = np.full((cm.rows, cols), np.inf)
    back = np.zeros((cm.rows, cols), dtype=int)

    start_cols = np.where(allowed & (np.abs(ys) <= center_corridor + 1e-9))[0]
    if start_cols.size == 0:
        start_cols = np.array([center_j])

    for j in start_cols:
        dp[0, j] = cost[0, j] if np.isfinite(cost[0, j]) else np.inf
    back[0, :] = np.arange(cols)

    for i in range(1, cm.rows):
        for j in range(cols):
            if not allowed[j]:
                continue
            best = np.inf
            best_k = j
            for k in range(max(0, j - 1), min(cols - 1, j + 1) + 1):
                prev = dp[i - 1, k]
                if not np.isfinite(prev):
                    continue
                total = prev + curv_weight * (j - k) ** 2 + cost[i, j]
                if total < best:
                    best = total
                    best_k = k
            dp[i, j] = best
            back[i, j] = best_k

    if not np.isfinite(dp[-1]).any():
        reach = int(np.isfinite(dp).any(axis=1).sum())
        return RoutePlan(
            s_axis=cm.s_axis,
            y_offset=np.zeros(cm.rows),
            feasible=False,
            cost_total=float("nan"),
            center_cost_total=float(np.nansum(cm.cost[:, center_j])),
            blocked_on_center=int(cm.blocked[:, center_j].sum()),
            min_clearance_m=0.0,
            max_cross_deg=float(np.nanmax(cm.cross_deg[:, allowed])),
            max_lateral_rate=0.0,
            notes=[f"仅前 {reach}/{cm.rows} 行存在可行状态"],
        )

    j_end = int(np.argmin(dp[-1]))
    path = np.zeros(cm.rows, dtype=int)
    path[-1] = j_end
    for i in range(cm.rows - 1, 0, -1):
        path[i - 1] = back[i, path[i]]

    y_offset = ys[path].astype(float)
    ds = float(cm.s_axis[1] - cm.s_axis[0]) if cm.rows > 1 else 0.15
    max_lateral_rate = float(np.mean(np.abs(np.diff(y_offset))) / ds) if cm.rows > 1 else 0.0
    total = float(dp[-1, j_end])
    center_cost_total = float(np.nansum(cm.cost[:, center_j]))
    blocked_on_center = int(cm.blocked[:, center_j].sum())

    clearance = np.full(cm.rows, np.nan)
    for i in range(cm.rows):
        bad = np.where(cm.blocked[i, :])[0]
        if bad.size == 0:
            continue
        clearance[i] = float(np.min(np.abs(ys[bad] - y_offset[i])))
    min_clearance = float(np.nanmin(clearance)) if np.isfinite(clearance).any() else float("inf")

    notes = []
    if blocked_on_center:
        notes.append(f"中线有 {blocked_on_center} 行被判定不可通行")
    if np.isfinite(clearance).any() and max(clearance[~np.isnan(clearance)], default=0.0) < 0.2:
        notes.append("局部安全余量小于 0.2 m，建议降速")

    return RoutePlan(
        s_axis=cm.s_axis,
        y_offset=y_offset,
        feasible=True,
        cost_total=total,
        center_cost_total=center_cost_total,
        blocked_on_center=blocked_on_center,
        min_clearance_m=min_clearance if np.isfinite(min_clearance) else 0.0,
        max_cross_deg=float(np.nanmax(cm.cross_deg[:, allowed])),
        max_lateral_rate=max_lateral_rate,
        notes=notes,
    )
