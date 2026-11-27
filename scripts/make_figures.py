"""生成报告用图表（全部输出 SVG 矢量图，便于入库与网页引用）。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from slopehaul.eval.scenarios import SCENARIOS  # noqa: E402
from slopehaul.sim.terrain import ALL_PROFILES  # noqa: E402

OUT = ROOT / "figures"
OUT.mkdir(exist_ok=True)
plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["svg.hashsalt"] = "slopehaul"
plt.rcParams["path.simplify_threshold"] = 1.0
plt.rcParams["svg.fonttype"] = "none"  # 文字用 <text> 输出，避免把每个字形转成路径
BLUE = "#2b6cb0"
RED = "#c53030"
GREY = "#4a5568"


def fig_terrain() -> Path:
    fig, ax = plt.subplots(figsize=(7.2, 3.4), dpi=160)
    for key, factory in ALL_PROFILES.items():
        prof = factory()
        s = np.linspace(0, prof.length, 90)
        z = np.array([prof.elevation(float(v)) for v in s])
        ax.plot(s, z, lw=1.6, label=key)
    ax.set_xlabel("沿行进方向弧长 s / m")
    ax.set_ylabel("高程 z / m")
    ax.set_title("五类评测地形纵断面")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7, loc="upper left")
    path = OUT / "figures_terrain_profiles.svg"
    fig.tight_layout()
    fig.savefig(path, metadata={"Date": None, "Creator": None})
    plt.close(fig)
    return path


def fig_s1() -> Path:
    import csv

    series = {}
    with (ROOT / "results" / "telemetry_S1_soft_climb.csv").open(encoding="utf-8") as f:
        for r in csv.DictReader(f):
            series.setdefault(r["policy"], []).append(r)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.2, 4.6), dpi=160, sharex=True)
    for policy, color in (("baseline", RED), ("proposed", BLUE)):
        rows = series.get(policy, [])
        if not rows:
            continue
        step = max(1, len(rows) // 60)
        s = [float(r["s"]) for r in rows][::step]
        slip = [float(r["slip_mean"]) * 100 for r in rows][::step]
        mu = [float(r["mu_est"]) for r in rows][::step]
        a1.plot(s, slip, color=color, lw=1.4, label=policy)
        a2.plot(s, mu, color=color, lw=1.4, label=policy)
    a1.axhline(35, ls="--", color=GREY, lw=0.9)
    a1.set_ylabel("平均滑移率 / %")
    a1.set_title("S1 松软土重载爬坡：滑移率 / 在线附着估计")
    a2.set_ylabel("在线 μ 估计")
    a2.set_xlabel("沿行进方向弧长 s / m")
    for ax in (a1, a2):
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)
    path = OUT / "figures_s1_slip.svg"
    fig.tight_layout()
    fig.savefig(path, metadata={"Date": None, "Creator": None})
    plt.close(fig)
    return path


def fig_s3() -> Path:
    import csv

    series = {}
    with (ROOT / "results" / "telemetry_S3_hill_hold.csv").open(encoding="utf-8") as f:
        for r in csv.DictReader(f):
            series.setdefault(r["policy"], []).append(r)
    fig, ax = plt.subplots(figsize=(7.2, 3.2), dpi=160)
    for policy, color in (("baseline", RED), ("proposed", BLUE)):
        rows = series.get(policy, [])
        if not rows:
            continue
        step = max(1, len(rows) // 60)
        t = [float(r["t"]) for r in rows][::step]
        v = [float(r["v_true"]) for r in rows][::step]
        ax.plot(t, v, color=color, lw=1.5, label=policy)
    ax.axhline(0, color=GREY, lw=0.8)
    ax.set_xlabel("时间 t / s")
    ax.set_ylabel("车速 v / (m/s)")
    ax.set_title("S3 18° 松软坡驻车：基线溜坡 vs 自适应驻车（负值为后溜）")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    path = OUT / "figures_s3_hillhold.svg"
    fig.tight_layout()
    fig.savefig(path, metadata={"Date": None, "Creator": None})
    plt.close(fig)
    return path


def fig_energy() -> Path:
    with (ROOT / "results" / "summary.json").open(encoding="utf-8") as f:
        summaries = json.load(f)
    keys = sorted({s["key"] for s in summaries})

    def get(key, policy):
        for s in summaries:
            if s["key"] == key and s["policy"] == policy:
                return s["energy_per_meter_wh"]
        return 0.0

    x = np.arange(len(keys))
    w = 0.36
    fig, ax = plt.subplots(figsize=(7.2, 3.2), dpi=160)
    ax.bar(x - w / 2, [get(k, "baseline") for k in keys], w, color=RED, label="基线")
    ax.bar(x + w / 2, [get(k, "proposed") for k in keys], w, color=BLUE, label="本方案")
    ax.set_xticks(x)
    ax.set_xticklabels([k.split("_")[0] + "\n" + k.split("_", 1)[1].replace("_", " ") for k in keys], fontsize=7)
    ax.set_ylabel("能耗 / (Wh/m)")
    ax.set_title("五场景单位里程能耗对比（越低越好）")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(fontsize=8)
    path = OUT / "figures_energy_compare.svg"
    fig.tight_layout()
    fig.savefig(path, metadata={"Date": None, "Creator": None})
    plt.close(fig)
    return path


def main() -> None:
    made = [fig_terrain(), fig_s1(), fig_s3(), fig_energy()]
    for p in made:
        print("saved", p.relative_to(ROOT))


if __name__ == "__main__":
    main()
