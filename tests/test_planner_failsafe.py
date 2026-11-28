"""新增模块的回归用例：代价层、横向规划、失效安全、载荷滑移模型。

运行： python tests/test_planner_failsafe.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from slopehaul.config import DEFAULT_CONFIG  # noqa: E402
from slopehaul.control.failsafe import FailsafeManager, Level  # noqa: E402
from slopehaul.estimation.load_slip import LoadSlipModel  # noqa: E402
from slopehaul.perception.costmap import build_costmap  # noqa: E402
from slopehaul.planning.lateral_planner import plan_route  # noqa: E402
from slopehaul.sim.terrain import TerrainProfile  # noqa: E402


class _Ridge:
    """14° 上坡窄道，右侧 s≥6 m 有 0.18 m 台阶。"""

    length = 12.0

    def height(self, s: float, y: float) -> float:
        z = np.tan(np.deg2rad(14.0)) * s + np.tan(np.deg2rad(6.0)) * y
        if s >= 6.0 and y >= 0.10:
            z += 0.18
        return float(z)

    def soil(self, s: float) -> float:
        return 1.0


def test_flat_terrain_has_no_blocked_cell():
    flat = TerrainProfile(name="flat", length=10.0)
    cm = build_costmap(flat, y_half=0.6)
    assert cm.blocked.sum() == 0, cm.stats()
    assert cm.stats()["max_slope_deg"] < 8.0, cm.stats()


def test_slope_correction_separates_step_from_ramp():
    cm = build_costmap(_Ridge(), y_half=0.75)
    j_right = int(np.argmin(np.abs(cm.y_axis - 0.45)))
    j_left = int(np.argmin(np.abs(cm.y_axis + 0.30)))
    wall = float(cm.step_m[(cm.s_axis >= 5.7) & (cm.s_axis <= 6.3), j_right].max())
    ramp = float(cm.step_m[(cm.s_axis >= 2.0) & (cm.s_axis <= 4.0), j_left].max())
    assert wall > 0.13, wall
    assert ramp < 0.03, ramp


def test_planner_detours_and_stays_out_of_blocked_cells():
    cm = build_costmap(_Ridge(), y_half=0.75)
    plan = plan_route(cm, y_limit=0.55)
    assert plan.feasible, plan.notes
    assert plan.blocked_on_center > 10, plan.blocked_on_center
    j_path = [int(np.argmin(np.abs(cm.y_axis - y))) for y in plan.y_offset]
    assert not cm.blocked[np.arange(cm.rows), j_path].any()
    assert float(plan.y_offset[-5:].mean()) < 0.0


def test_failsafe_ladder_and_estop_latch():
    fs = FailsafeManager()
    assert fs.step(0.0, 0.05, 1.0, 0.0, 0.0, 4.0, 40.0).level == Level.OK
    assert fs.step(1.0, 0.9, 1.0, 0.0, 0.0, 4.0, 40.0).level == Level.HOLD
    assert fs.step(2.0, 0.05, 1.0, 0.0, 0.0, 4.0, 78.0).level == Level.DERATE
    stop = fs.step(3.0, 0.05, 1.0, 0.0, 0.0, 4.0, 40.0, estop=True)
    assert stop.level == Level.STOP and stop.latched
    assert fs.step(4.0, 0.05, 1.0, 0.0, 0.0, 4.0, 40.0).level == Level.STOP
    assert not fs.clear_estop(operator_ack=False)
    assert fs.clear_estop(operator_ack=True)
    assert fs.step(5.0, 0.05, 1.0, 0.0, 0.0, 4.0, 40.0).level == Level.OK


def test_load_slip_model_recovers_linear_relation():
    rng = np.random.default_rng(0)
    grade = rng.uniform(5.0, 20.0, 60)
    mu = rng.uniform(0.45, 0.80, 60)
    speed = rng.uniform(0.2, 1.2, 60)
    demand = np.abs(np.tan(np.deg2rad(grade))) / mu
    slip = 0.02 + 0.25 * demand - 0.05 * speed
    model = LoadSlipModel()
    report = model.fit(grade, mu, speed, slip)
    assert report.r2 > 0.95, report.summary()
    assert report.mae < 0.01
    pred = model.predict(grade, mu, speed)
    assert np.allclose(pred, np.clip(slip, 0.0, 1.0), atol=0.02)


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
