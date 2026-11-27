"""Smoke tests: 感知、估计、控制与闭环仿真的最小回归集合。

运行： python tests/test_smoke.py    （或 python -m pytest tests -q）
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from slopehaul.config import DEFAULT_CONFIG, ScenarioParams  # noqa: E402
from slopehaul.estimation.slip import FrictionRLS, SlipEstimator  # noqa: E402
from slopehaul.perception.elevation import terrain_features  # noqa: E402
from slopehaul.perception.soil_classifier import SoilClassifier, classify_soil  # noqa: E402
from slopehaul.perception.traversability import evaluate  # noqa: E402
from slopehaul.sim.sensors import SensorSuite  # noqa: E402
from slopehaul.sim.terrain import hill_hold_profile, rough_downhill_profile, soft_slope_profile, steep_wet_profile  # noqa: E402
from slopehaul.sim.world import Simulation  # noqa: E402


def _features(terrain, s: float, cfg=DEFAULT_CONFIG, seed: int = 3):
    suite = SensorSuite(cfg.perception, cfg.vehicle, seed=seed)
    frame = suite.read(0.0, s, 0.0, terrain, 0.0, 0.0, 0.0, 0.0, 0.0, np.zeros(4), 0.0, 180.0)
    return terrain_features(
        frame.elevation, cfg.perception.grid_res, cfg.perception.plane_window, cfg.perception.footprint_margin
    )


def test_grade_estimation_matches_profile():
    """平面拟合出的纵坡应与地形设定值一致（误差 < 1.5°）。"""
    feats = _features(hill_hold_profile(), 3.0)
    assert abs(abs(feats.grade_deg) - 18.0) < 1.5, feats.grade_deg


def test_roughness_positive_on_rough_terrain():
    """起伏地形应给出非零粗糙度，且高于平滑地形。"""
    rough = _features(rough_downhill_profile(), 6.0, seed=4)
    smooth = _features(soft_slope_profile(), 6.0, seed=4)
    assert rough.roughness > 0.02, rough.roughness
    assert rough.roughness >= smooth.roughness, (rough.roughness, smooth.roughness)


def test_rls_respects_bounds():
    """附着系数在线估计必须落在合理区间内。"""
    rls = FrictionRLS(slip_opt=0.18, init=0.7)
    for _ in range(200):
        rls.update(slip=0.2, mu_util=1.2, weight=1.0)
    assert 0.25 <= rls.mu_max <= 0.88, rls.mu_max


def test_slip_estimator_zero_when_rolling():
    """轮速与地速一致时滑移率应接近 0。"""
    est = SlipEstimator(DEFAULT_CONFIG.vehicle)
    out = None
    for _ in range(40):
        out = est.update(0.02, np.full(4, 1.0 / 0.24), 0.0, 1.0, np.full(4, 1500.0), 100.0, 0.0, 500.0)
    assert out.slip_ratio_mean < 0.05, out.slip_ratio_mean


def test_steep_wet_is_blocked():
    """26° 湿滑坡应被判为不可通行。"""
    cfg = DEFAULT_CONFIG
    terrain = steep_wet_profile()
    feats = _features(terrain, 7.0, seed=5)
    soil = classify_soil(SoilClassifier(None), np.full((32, 32), 0.30), feats.slope_deg, feats.roughness)
    trav = evaluate(feats, soil, cfg.perception, cfg.control, mu_online=0.50, speed=0.4, load=120.0, curvature=0.0, lookahead=2.5)
    assert trav.blocked, (feats.slope_deg, trav.block_reason)


def test_soft_slope_is_traversable():
    """20° 松软坡在视觉先验下不应被判死（避免过度保守）。"""
    cfg = DEFAULT_CONFIG
    feats = _features(soft_slope_profile(), 9.0, seed=6)
    soil = classify_soil(SoilClassifier(None), np.full((32, 32), 0.52), feats.slope_deg, feats.roughness)
    trav = evaluate(feats, soil, cfg.perception, cfg.control, mu_online=0.62, speed=0.8, load=180.0, curvature=0.0, lookahead=4.0)
    assert not trav.blocked, (feats.slope_deg, trav.block_reason)
    assert trav.v_limit > 0.2


def test_closed_loop_climbs_soft_slope():
    """本方案应能在松软土坡上完成 12 m 运输（短程冒烟测试）。"""
    spec = ScenarioParams(name="smoke", length=12.0, payload=180.0, duration=25.0)
    sim = Simulation(DEFAULT_CONFIG, soft_slope_profile(), policy="proposed", payload=180.0, seed=1)
    result = sim.run(spec)
    assert result.summary["reached"], result.summary["final_s"]


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                failures += 1
                print(f"FAIL {name}: {exc}")
    raise SystemExit(1 if failures else 0)
