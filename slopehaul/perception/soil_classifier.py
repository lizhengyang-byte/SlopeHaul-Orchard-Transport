"""地形类型识别：小型 MLP + 可解释特征，输出土壤类型与附着系数先验。

输入不是原始图像，而是 8 维可解释特征（亮度、纹理能量、熵、坡度、粗糙度等），
这样在嵌入式控制器（<1 ms）上可实时运行，也便于现场排查误判。
真实部署时应替换为相机 RGB 裁剪 + 与本文相同的特征层，模型结构与训练流程不变。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

SOIL_NAMES = ("压实土", "草地", "松软土", "湿滑黏土")
# 峰值附着系数先验（动摩擦，含水率中等）
SOIL_MU_PRIOR = np.array([0.80, 0.66, 0.62, 0.45])
# 滚动阻力先验
SOIL_CRR_PRIOR = np.array([0.030, 0.055, 0.095, 0.080])
# 允许通行的坡度上限（按土壤类型）
SOIL_SLOPE_LIMIT = np.array([28.0, 25.0, 22.0, 15.0])


def appearance_features(img: np.ndarray, slope_deg: float, roughness: float) -> np.ndarray:
    """从地表纹理图提取 8 维特征。"""
    img = np.asarray(img, dtype=float)
    mean = float(img.mean())
    std = float(img.std())
    gx = np.diff(img, axis=1)
    gy = np.diff(img, axis=0)
    grad = float(np.mean(np.abs(gx))) + float(np.mean(np.abs(gy)))
    lap = float(np.mean(np.abs(np.gradient(np.gradient(img, axis=0), axis=0))))
    hist, _ = np.histogram(img, bins=16, range=(0.0, 1.0), density=False)
    p = hist / max(hist.sum(), 1)
    ent = float(-np.sum(p[p > 0] * np.log2(p[p > 0])) / 4.0)
    dark = float(np.mean(img < 0.35))
    return np.array([mean, std, grad, lap, ent, dark, slope_deg / 30.0, roughness / 0.08])


class SoilClassifier:
    """两层 MLP（8-16-4），权重以 json 保存；缺失时退化为规则分类器。"""

    def __init__(self, model_path: str | Path | None = None) -> None:
        self.w1 = self.b1 = self.w2 = self.b2 = None
        self.model_path = Path(model_path) if model_path else None
        self.loaded = False
        if self.model_path and self.model_path.exists():
            if self.model_path.suffix == ".json":
                # JSON 权重（纯文本，便于入库与审阅，不依赖二进制 npz）
                import json

                data = json.loads(self.model_path.read_text(encoding="utf-8"))
                arr = lambda k: np.asarray(data[k], dtype=float)  # noqa: E731
                self.w1, self.b1 = arr("w1"), arr("b1")
                self.w2, self.b2 = arr("w2"), arr("b2")
                self.mean, self.scale = arr("mean"), arr("scale")
            else:
                data = np.load(self.model_path)
                self.w1, self.b1 = data["w1"], data["b1"]
                self.w2, self.b2 = data["w2"], data["b2"]
                self.mean, self.scale = data["mean"], data["scale"]
            self.loaded = True
        else:
            self.mean = np.zeros(8)
            self.scale = np.ones(8)

    # ------------------------------------------------------------------
    def _forward(self, x: np.ndarray) -> np.ndarray:
        z = (x - self.mean) / self.scale
        h = np.tanh(z @ self.w1 + self.b1)
        logits = h @ self.w2 + self.b2
        logits -= logits.max()
        e = np.exp(logits)
        return e / e.sum()

    def predict_proba(self) -> np.ndarray:  # pragma: no cover - 占位避免误用
        raise NotImplementedError

    def classify(self, features: np.ndarray) -> tuple[int, np.ndarray]:
        if self.loaded:
            proba = self._forward(features)
            return int(np.argmax(proba)), proba
        # 规则回退：暗+平滑 → 湿滑；高纹理+中亮 → 松软；高纹理 + 亮 → 草地；亮 + 低纹理 → 压实土
        f = features
        if f[0] < 0.36 and f[1] < 0.045:
            cls = 3
        elif f[1] > 0.09 and f[0] < 0.52:
            cls = 1
        elif f[1] > 0.05:
            cls = 2
        else:
            cls = 0
        proba = np.zeros(4)
        proba[cls] = 0.85
        proba += 0.0375
        return cls, proba


@dataclass
class SoilReport:
    soil_class: int
    soil_name: str
    proba: np.ndarray
    mu_prior: float
    crr_prior: float
    slope_limit: float


def classify_soil(clf: SoilClassifier, img: np.ndarray, slope_deg: float, roughness: float) -> SoilReport:
    feats = appearance_features(img, slope_deg, roughness)
    cls, proba = clf.classify(feats)
    return SoilReport(
        soil_class=cls,
        soil_name=SOIL_NAMES[cls],
        proba=proba,
        mu_prior=float(SOIL_MU_PRIOR[cls]),
        crr_prior=float(SOIL_CRR_PRIOR[cls]),
        slope_limit=float(SOIL_SLOPE_LIMIT[cls]),
    )


# ---------------------------------------------------------------------------
# 训练（numpy 实现，无需 PyTorch / GPU）
# ---------------------------------------------------------------------------


def _synthetic_batch(rng: np.random.Generator, n: int) -> tuple[np.ndarray, np.ndarray]:
    from ..sim.sensors import SensorSuite
    from ..config import PerceptionParams, VehicleParams

    suite = SensorSuite(PerceptionParams(), VehicleParams(), seed=int(rng.integers(0, 1 << 30)))
    suite.rng = rng
    xs, ys = [], []
    for _ in range(n):
        cls = int(rng.integers(0, 4))
        img = suite._appearance(cls)
        # 加入成像噪声（曝光漂移 + 传感器噪声），避免合成集过度干净导致验证准确率虚高
        img = np.clip(img * rng.uniform(0.90, 1.10) + rng.normal(0.0, 0.015, img.shape), 0.0, 1.0)
        # 与类别相关的坡度/粗糙度采样，避免特征泄漏
        slope = rng.uniform(2, 26) if cls == 0 else rng.uniform(2, 20)
        rough = rng.uniform(0.015, 0.035) if cls == 0 else rng.uniform(0.02, 0.08)
        xs.append(appearance_features(img, slope, rough))
        ys.append(cls)
    return np.array(xs), np.array(ys)


def train_classifier(n_train: int = 4000, n_val: int = 800, seed: int = 7) -> dict:
    rng = np.random.default_rng(seed)
    x, y = _synthetic_batch(rng, n_train)
    xv, yv = _synthetic_batch(rng, n_val)

    mean = x.mean(axis=0)
    scale = x.std(axis=0) + 1e-6
    xt = (x - mean) / scale
    xvt = (xv - mean) / scale

    hidden = 16
    w1 = rng.normal(0, 0.5, size=(x.shape[1], hidden))
    b1 = np.zeros(hidden)
    w2 = rng.normal(0, 0.5, size=(hidden, 4))
    b2 = np.zeros(4)
    params = [w1, b1, w2, b2]
    m = [np.zeros_like(p) for p in params]
    vvoid = [np.zeros_like(p) for p in params]
    lr, beta1, beta2, eps = 0.02, 0.9, 0.999, 1e-8

    onehot = np.eye(4)[y]
    for step in range(1, 1201):
        idx = rng.integers(0, xt.shape[0], size=128)
        xb, yb = xt[idx], onehot[idx]
        h_pre = xb @ w1 + b1
        h = np.tanh(h_pre)
        logits = h @ w2 + b2
        logits -= logits.max(axis=1, keepdims=True)
        p = np.exp(logits)
        p /= p.sum(axis=1, keepdims=True)

        g_logits = (p - yb) / len(idx)
        g_w2 = h.T @ g_logits
        g_b2 = g_logits.sum(axis=0)
        g_h = g_logits @ w2.T * (1 - h ** 2)
        g_w1 = xb.T @ g_h
        g_b1 = g_h.sum(axis=0)
        grads = [g_w1, g_b1, g_w2, g_b2]

        for i, (param, grad) in enumerate(zip(params, grads)):
            m[i] = beta1 * m[i] + (1 - beta1) * grad
            vvoid[i] = beta2 * vvoid[i] + (1 - beta2) * grad ** 2
            mhat = m[i] / (1 - beta1 ** step)
            vhat = vvoid[i] / (1 - beta2 ** step)
            param -= lr * mhat / (np.sqrt(vhat) + eps)

    h = np.tanh(xvt @ w1 + b1)
    logits = h @ w2 + b2
    acc = float(np.mean(np.argmax(logits, axis=1) == yv))
    conf = np.zeros((4, 4), dtype=int)
    for t, pr in zip(yv, np.argmax(logits, axis=1)):
        conf[t, pr] += 1
    return {
        "w1": w1, "b1": b1, "w2": w2, "b2": b2,
        "mean": mean, "scale": scale,
        "val_acc": acc, "confusion": conf,
    }
