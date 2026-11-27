"""批量评测：同一场景下跑基线与本方案，输出汇总表与逐帧遥测。"""

from __future__ import annotations

import csv
import json
from pathlib import Path

from ..config import Config, DEFAULT_CONFIG
from ..sim.world import Simulation
from .metrics import compare_rows, summarize_improvement
from .scenarios import SCENARIOS, ScenarioSpec


def run_all(
    cfg: Config | None = None,
    policies: tuple[str, ...] = ("baseline", "proposed"),
    keys: list[str] | None = None,
    soil_model_path: str | None = None,
    sensor_dropout: bool = False,
) -> list[dict]:
    cfg = cfg or DEFAULT_CONFIG
    keys = keys or list(SCENARIOS)
    runs = []
    for key in keys:
        spec: ScenarioSpec = SCENARIOS[key]
        terrain = spec.terrain_factory()
        for policy in policies:
            sim = Simulation(
                cfg,
                terrain,
                policy=policy,
                payload=spec.params.payload,
                soil_model_path=soil_model_path,
                sensor_dropout=sensor_dropout,
            )
            result = sim.run(spec.params)
            result.summary["policy"] = policy
            result.summary["key"] = key
            result.summary["description"] = spec.description
            result.summary["focus"] = spec.focus
            result.summary["outcome"] = spec.outcome(result.summary)
            runs.append({"key": key, "policy": policy, "spec": spec, "result": result})
    return runs


def write_outputs(runs: list[dict], out_dir: str | Path = "results") -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    summaries = [run["result"].summary for run in runs]
    with (out / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summaries, f, ensure_ascii=False, indent=2)

    # 逐帧遥测：每个场景一个 CSV，两策略各自一列前缀
    for key in sorted({run["key"] for run in runs}):
        rows_by_policy = {run["policy"]: run["result"].rows for run in runs if run["key"] == key}
        path = out / f"telemetry_{key}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            header = ["policy", "t", "s", "v_true", "slip_mean", "mu_true", "mu_est", "roll", "torque", "brake", "v_limit"]
            writer.writerow(header)
            for policy in ("baseline", "proposed"):
                rows = rows_by_policy.get(policy, [])
                for r in rows:
                    writer.writerow([policy] + [r[c] for c in header[1:]])

    # 对比表（Markdown）
    pairs = []
    for key in sorted({run["key"] for run in runs}):
        s = [run["result"].summary for run in runs if run["key"] == key]
        if len(s) == 2:
            pairs.append(compare_rows(s)[0])

    md = _render_markdown(pairs)
    (out / "comparison.md").write_text(md, encoding="utf-8")

    agg = summarize_improvement(pairs) if pairs else {}
    with (out / "aggregate.json").open("w", encoding="utf-8") as f:
        json.dump(agg, f, ensure_ascii=False, indent=2)

    return {"summaries": summaries, "pairs": pairs, "aggregate": agg, "markdown": md}


def _render_markdown(pairs: list[dict]) -> str:
    if not pairs:
        return "# 对比结果\n\n无可用数据。\n"
    lines = [
        "# 基线与本方案对比",
        "",
        "| 场景 | 指标 | 基线（定速+均分扭矩） | 本方案（地形感知+自适应） |",
        "|---|---|---|---|",
    ]
    for p in pairs:
        b, q = p["baseline"], p["proposed"]
        cn = {"reached": "到达终点", "safe_stop": "安全拦停", "failed": "任务失败", "tipover": "倾翻"}
        rows = [
            ("任务结论", cn.get(b.get("outcome", ""), ""), cn.get(q.get("outcome", ""), "")),
            ("通过场景", b["reached"], q["reached"]),
            ("耗时 (s)", f"{b['time_s']:.1f}", f"{q['time_s']:.1f}"),
            ("平均滑移率", f"{b['mean_slip'] * 100:.1f}%", f"{q['mean_slip'] * 100:.1f}%"),
            ("P95 滑移率", f"{b['p95_slip'] * 100:.1f}%", f"{q['p95_slip'] * 100:.1f}%"),
            ("严重打滑占比", f"{b['slip_above_035_ratio'] * 100:.1f}%", f"{q['slip_above_035_ratio'] * 100:.1f}%"),
            ("最大侧倾角 (°)", f"{b['max_roll']:.1f}", f"{q['max_roll']:.1f}"),
            ("倾翻", b["tipped"], q["tipped"]),
            ("后溜距离 (m)", f"{b['back_slip_distance']:.2f}", f"{q['back_slip_distance']:.2f}"),
            ("驻车溜坡 (m)", f"{b['hold_slip_distance']:.2f}", f"{q['hold_slip_distance']:.2f}"),
            ("附着估计 MAE", f"{b['mu_est_mae']:.3f}", f"{q['mu_est_mae']:.3f}"),
            ("能耗 (Wh/m)", f"{b['energy_per_meter_wh']:.3f}", f"{q['energy_per_meter_wh']:.3f}"),
            ("安全介入", b["safety_interventions"], q["safety_interventions"]),
        ]
        for name, bv, qv in rows:
            bstr = "是" if bv is True else ("否" if bv is False else str(bv))
            qstr = "是" if qv is True else ("否" if qv is False else str(qv))
            lines.append(f"| {p['scenario']} | {name} | {bstr} | {qstr} |")
    lines.append("")
    return "\n".join(lines)
