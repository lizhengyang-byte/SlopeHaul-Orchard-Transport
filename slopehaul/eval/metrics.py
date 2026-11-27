"""评测指标与对比汇总。"""

from __future__ import annotations

import numpy as np

KEY_METRICS = [
    ("reached", "通过场景", lambda r: "是" if r["reached"] else "否"),
    ("time_s", "耗时 (s)", lambda r: f"{r['time_s']:.1f}"),
    ("mean_slip", "平均滑移率", lambda r: f"{r['mean_slip'] * 100:.1f}%"),
    ("p95_slip", "P95 滑移率", lambda r: f"{r['p95_slip'] * 100:.1f}%"),
    ("slip_above_035_ratio", "严重打滑时间占比", lambda r: f"{r['slip_above_035_ratio'] * 100:.1f}%"),
    ("max_roll", "最大侧倾角 (°)", lambda r: f"{r['max_roll']:.1f}"),
    ("tipped", "是否倾翻", lambda r: "是" if r["tipped"] else "否"),
    ("back_slip_distance", "后溜距离 (m)", lambda r: f"{r['back_slip_distance']:.2f}"),
    ("hold_slip_distance", "驻车溜坡 (m)", lambda r: f"{r['hold_slip_distance']:.2f}"),
    ("mu_est_mae", "附着估计 MAE", lambda r: f"{r['mu_est_mae']:.3f}"),
    ("energy_per_meter_wh", "能耗 (Wh/m)", lambda r: f"{r['energy_per_meter_wh']:.3f}"),
    ("safety_interventions", "安全介入次数", lambda r: str(r["safety_interventions"])),
]


def compare_rows(summaries: list[dict]) -> list[dict]:
    """把两次运行（基线 / 本方案）压成一行对比。"""
    by_policy = {s["policy"]: s for s in summaries}
    if "baseline" not in by_policy or "proposed" not in by_policy:
        return summaries
    b, p = by_policy["baseline"], by_policy["proposed"]
    return [{
        "scenario": p["scenario"],
        "terrain": p["terrain"],
        "baseline": b,
        "proposed": p,
    }]


def metric_delta(metric: str, lower_is_better: bool = True):
    def _fmt(pair: dict) -> str:
        b, p = pair["baseline"][metric], pair["proposed"][metric]
        if isinstance(b, bool) or isinstance(b, str):
            return f"{b} → {p}"
        if b == 0:
            return f"{b} → {p}"
        delta = (p - b) / abs(b) * 100
        arrow = "↓" if delta < 0 else "↑"
        return f"{b:.3g} → {p:.3g} ({arrow}{abs(delta):.0f}%)"
    return _fmt


def summarize_improvement(pairs: list[dict]) -> dict:
    slips = [p["proposed"]["mean_slip"] for p in pairs]
    base_slips = [p["baseline"]["mean_slip"] for p in pairs]
    return {
        "scenarios": len(pairs),
        "mean_slip_baseline": float(np.mean(base_slips)),
        "mean_slip_proposed": float(np.mean(slips)),
        "slip_reduction_pct": float((1 - np.mean(slips) / max(np.mean(base_slips), 1e-6)) * 100),
        "tipped_baseline": int(sum(p["baseline"]["tipped"] for p in pairs)),
        "tipped_proposed": int(sum(p["proposed"]["tipped"] for p in pairs)),
        "reached_baseline": int(sum(p["baseline"]["reached"] for p in pairs)),
        "reached_proposed": int(sum(p["proposed"]["reached"] for p in pairs)),
    }
