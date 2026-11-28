"""载荷/坡度滑移代理模型：滑移 ≈ w0 + w1·(tanθ/μ) + w2·v。

滑移率是比率量，理论质量约掉；载荷通过质心高、轴荷转移与土壤压实间接影响（见 sim/vehicle.py）。
拟合结果同时输出 R² 与 MAE，不在报告里藏“模型不可信”这件事。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class SlipFitReport:
    coef: np.ndarray
    r2: float
    mae: float
    n_samples: int

    def summary(self) -> dict:
        return {
            "coef": [round(float(c), 5) for c in self.coef],
            "r2": round(float(self.r2), 4),
            "mae": round(float(self.mae), 4),
            "n_samples": int(self.n_samples),
        }


class LoadSlipModel:
    def __init__(self) -> None:
        self.coef: np.ndarray | None = None

    @staticmethod
    def features(grade_deg, mu, speed_mps) -> np.ndarray:
        g = np.asarray(grade_deg, dtype=float)
        mu = np.maximum(np.asarray(mu, dtype=float), 0.05)
        v = np.asarray(speed_mps, dtype=float)
        demand = np.abs(np.tan(np.deg2rad(g))) / mu
        return np.column_stack([np.ones_like(g), demand, v])

    def fit(self, grade_deg, mu, speed_mps, slip_ratio) -> SlipFitReport:
        X = self.features(grade_deg, mu, speed_mps)
        y = np.clip(np.asarray(slip_ratio, dtype=float), 0.0, 1.0)
        coef, *_ = np.linalg.lstsq(X, y, rcond=None)
        self.coef = coef
        pred = np.clip(X @ coef, 0.0, 1.0)
        ss_res = float(np.sum((y - pred) ** 2))
        ss_tot = float(np.sum((y - y.mean()) ** 2)) + 1e-12
        return SlipFitReport(coef=coef, r2=1.0 - ss_res / ss_tot, mae=float(np.mean(np.abs(y - pred))), n_samples=int(y.size))

    def predict(self, grade_deg, mu, speed_mps) -> np.ndarray:
        if self.coef is None:
            raise RuntimeError("模型尚未拟合")
        return np.clip(self.features(grade_deg, mu, speed_mps) @ self.coef, 0.0, 1.0)

    def grade_sensitivity(self, grade_deg: float, mu: float, speed_mps: float) -> float:
        """坡度每增加 1° 的滑移率增量。"""

        if self.coef is None:
            raise RuntimeError("模型尚未拟合")
        return float(self.predict(grade_deg + 1.0, mu, speed_mps)[0]) - float(self.predict(grade_deg, mu, speed_mps)[0])
