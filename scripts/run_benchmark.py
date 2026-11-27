#!/usr/bin/env python3
"""批量评测入口：五种地形 × 两种策略，输出 results/ 下的汇总与遥测。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from slopehaul.eval.benchmark import run_all, write_outputs  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description="SlopeHaul 坡地运输基准评测")
    ap.add_argument("--scenarios", nargs="*", default=None, help="场景 key，缺省为全部")
    ap.add_argument("--out", default=str(ROOT / "results"), help="输出目录")
    ap.add_argument("--soil-model", default=str(ROOT / "models" / "soil_mlp.json"))
    ap.add_argument("--noise", action="store_true", help="开启深度缺失/强噪声压力测试")
    args = ap.parse_args()

    model_path = args.soil_model if Path(args.soil_model).exists() else None
    if model_path is None:
        # 权重文件不在仓库里（纯文本 JSON，可由训练脚本确定性复现）：首次运行自动训练
        from slopehaul.perception.soil_classifier import train_classifier

        print("未找到土壤分类权重，正在按固定种子训练 ...")
        Path(args.soil_model).parent.mkdir(parents=True, exist_ok=True)
        result = train_classifier()
        import numpy as _np

        payload = {k: _np.asarray(v).round(6).tolist() for k, v in result.items() if isinstance(v, _np.ndarray)}
        Path(args.soil_model).write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        model_path = args.soil_model
        print(f"权重已保存至 {model_path}（验证集准确率 {result['val_acc']:.3f}）")
    runs = run_all(keys=args.scenarios, soil_model_path=model_path, sensor_dropout=args.noise)
    out = write_outputs(runs, args.out)

    print(f"场景数：{len(out['pairs'])}，输出目录：{args.out}")
    print(json.dumps(out["aggregate"], ensure_ascii=False, indent=2))
    print("\n" + out["markdown"])


if __name__ == "__main__":
    main()
