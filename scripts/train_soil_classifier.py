#!/usr/bin/env python3
"""训练 8-16-4 土壤分类 MLP，并把权重导出为 JSON（文本，便于入库）。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from slopehaul.perception.soil_classifier import train_classifier  # noqa: E402


def main() -> None:
    out = ROOT / "models" / "soil_mlp.json"
    out.parent.mkdir(exist_ok=True)
    result = train_classifier(n_train=6000, n_val=1200, seed=7)
    payload = {k: np.asarray(v).round(6).tolist() for k, v in result.items() if isinstance(v, np.ndarray)}
    out.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    print(f"验证集准确率 {result['val_acc']:.3f}，混淆矩阵\n{result['confusion']}")
    print(f"权重已写入 {out}")


if __name__ == "__main__":
    main()
