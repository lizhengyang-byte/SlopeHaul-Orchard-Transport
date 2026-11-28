"""演示：2.5D 代价层 + 重心约束横向规划 + 失效安全阶梯 + 载荷滑移模型。

运行： python scripts/run_planner_demo.py
输出： results/planner_demo.json、figures/figures_costmap_route.svg
不改变已冻结的 S1–S5 基准，只在一个“窄道 + 单侧台阶”的合成地形上验证新模块。
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from slopehaul.control.failsafe import FailsafeManager  # noqa: E402
from slopehaul.estimation.load_slip import LoadSlipModel  # noqa: E402
from slopehaul.perception.costmap import build_costmap  # noqa: E402
from slopehaul.planning.lateral_planner import plan_route  # noqa: E402
from slopehaul.sim.terrain import hill_hold_profile, soft_slope_profile  # noqa: E402

plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
plt.rcParams["svg.hashsalt"] = "slopehaul"


class NarrowRidge:
    """14° 上坡窄道（可用宽度约 1.1 m），右侧 s≥6 m 处有一级 0.18 m 台阶。"""

    length = 14.0
    ramp = float(np.tan(np.deg2rad(14.0)))
    cross = float(np.tan(np.deg2rad(6.0)))

    def height(self, s: float, y: float) -> float:
        z = self.ramp * s + self.cross * y
        z += 0.04 * (0.7 * np.sin(2 * np.pi * s / 1.6 + 1.3 * y) + 0.5 * np.sin(2 * np.pi * (s + 0.8 * y) / 1.1))
        if s >= 6.0 and y >= 0.10:
            z += 0.18
        return float(z)

    def soil(self, s: float) -> float:
        return 1.0


def load_telemetry(name: str) -> list[dict]:
    with (ROOT / "results" / f"telemetry_{name}.csv").open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main() -> None:
    terrain = NarrowRidge()
    cm = build_costmap(terrain, y_half=0.75, res=0.15)
    plan = plan_route(cm, y_limit=0.55, curv_weight=0.35)

    j_right = int(np.argmin(np.abs(cm.y_axis - 0.45)))
    j_left = int(np.argmin(np.abs(cm.y_axis + 0.30)))
    rows_wall = (cm.s_axis >= 5.7) & (cm.s_axis <= 6.3)
    step_wall = float(cm.step_m[rows_wall, j_right].max())
    rows_ramp = (cm.s_axis >= 2.5) & (cm.s_axis <= 3.5)
    step_ramp = float(cm.step_m[rows_ramp, j_left].max())

    fs = FailsafeManager()
    events: list[dict] = []
    for k in range(31):
        t = k * 0.5
        link_age = 0.05
        temp = 40.0
        estop = False
        if 6.0 <= t < 8.0:
            link_age = 0.9
        if 10.0 <= t < 12.0:
            temp = 78.0
        if 12.0 <= t < 14.0:
            estop = True
        act = fs.step(t, link_age, s=min(14.0, 0.6 * t), y=-0.2, roll_deg=3.0, pitch_deg=12.0, motor_temp_c=temp, estop=estop)
        events.append({"t": t, **act.summary()})

    transitions = []
    prev = None
    for e in events:
        if e["level"] != prev:
            transitions.append({"t": e["t"], "level": e["level"], "reasons": e["reasons"]})
            prev = e["level"]
    acked = fs.clear_estop(operator_ack=True)
    after_ack = fs.step(15.0, 0.05, 12.0, -0.2, 3.0, 12.0, 40.0).summary()

    model = LoadSlipModel()
    grades, mus, speeds, slips = [], [], [], []
    for name, prof in (("S1_soft_climb", soft_slope_profile()), ("S3_hill_hold", hill_hold_profile())):
        for row in load_telemetry(name)[::25]:
            grades.append(prof.grade(float(row["s"])))
            mus.append(float(row["mu_true"]))
            speeds.append(float(row["v_true"]))
            slips.append(float(row["slip_mean"]))
    report = model.fit(grades, mus, speeds, slips)

    figure_path = ROOT / "figures" / "figures_costmap_route.svg"
    figure_path.parent.mkdir(exist_ok=True)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.6, 5.2), dpi=160, gridspec_kw={"height_ratios": [3, 1]})
    ax1.imshow(
        cm.cost.T,
        origin="lower",
        aspect="auto",
        extent=[cm.s_axis[0], cm.s_axis[-1], cm.y_axis[0], cm.y_axis[-1]],
        cmap="magma",
        vmin=0.0,
        vmax=1.0,
    )
    ax1.plot(cm.s_axis, np.zeros_like(cm.s_axis), ls="--", lw=1.0, color="#7fd4ff", label="中线（未规划）")
    ax1.plot(plan.s_axis, plan.y_offset, lw=1.8, color="#1fe08a", label="CoM 约束规划轨迹")
    ax1.set_ylabel("横向偏移 y / m")
    ax1.set_title("窄道 2.5D 代价层与规划轨迹（右侧台阶被膨胀区禁行）")
    ax1.legend(fontsize=7, loc="upper left")
    fig.colorbar(ax1.images[0], ax=ax1, fraction=0.03, pad=0.02, label="代价")
    ax2.plot(cm.s_axis, np.zeros_like(cm.s_axis), ls="--", lw=1.0, color="#888")
    ax2.plot(plan.s_axis, plan.y_offset, lw=1.8, color="#1fe08a")
    ax2.set_xlabel("沿行进方向弧长 s / m")
    ax2.set_ylabel("y / m")
    ax2.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(figure_path, metadata={"Date": None, "Creator": None})
    plt.close(fig)

    out = {
        "terrain": "narrow_ridge_14deg_step0.18m",
        "costmap": cm.stats(),
        "plan": plan.summary(),
        "plan_notes": plan.notes,
        "step_detection": {
            "step_at_ledge_m": round(step_wall, 4),
            "step_on_ramp_m": round(step_ramp, 4),
            "comment": "同一高差在纯斜坡上被坡度校正扣掉，只有孤立落差被判为台阶",
        },
        "failsafe": {"transitions": transitions, "operator_ack_accepted": acked, "after_ack": after_ack, "levels_seen": sorted({e["level"] for e in events})},
        "load_slip": report.summary(),
        "slip_sensitivity_per_degree": round(model.grade_sensitivity(14.0, 0.62, 0.5), 4),
        "figure": str(figure_path.relative_to(ROOT)),
    }
    (ROOT / "results" / "planner_demo.json").write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")

    print("costmap:", {k: round(v, 4) for k, v in cm.stats().items()})
    print("plan:", plan.summary())
    print("step wall/ramp:", round(step_wall, 4), round(step_ramp, 4))
    print("failsafe levels:", out["failsafe"]["levels_seen"], "ack:", acked)
    print("load-slip:", report.summary())
    print("slip sensitivity per degree:", out["slip_sensitivity_per_degree"])


if __name__ == "__main__":
    main()
