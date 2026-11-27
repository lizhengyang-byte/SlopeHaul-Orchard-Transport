"""山地果园地形模型。

地形以沿运输路径的弧长 s 参数化：纵向坡度、横坡、峰值附着系数、粗糙度、曲率
都可以随 s 变化。高程通过坡度积分得到，因此感知模块采样到的坡面与车辆动力学
使用的地形完全一致（同一份 ground truth），便于做“感知误差—控制效果”的因果分析。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np


def _const(value: float) -> Callable[[float], float]:
    return lambda s: value


@dataclass
class TerrainProfile:
    """一条运输路径的地形描述。"""

    name: str
    length: float
    grade_deg: Callable[[float], float] = _const(0.0)      # 纵坡，正值为上坡
    cross_deg: Callable[[float], float] = _const(0.0)      # 横坡
    mu_max: Callable[[float], float] = _const(0.75)        # 峰值附着系数
    roughness: Callable[[float], float] = _const(0.02)     # 粗糙度幅值 m
    curvature: Callable[[float], float] = _const(0.0)      # 1/m，正值为左转
    soil_class: Callable[[float], float] = _const(0.0)     # 0 硬土 1 草地 2 松软 3 湿滑
    width: float = 1.9

    def __post_init__(self) -> None:
        self._ds = 0.05
        self._s = np.arange(0.0, self.length + 4.0, self._ds)
        self._g = np.array([self.grade_deg(float(s)) for s in self._s])
        self._c = np.array([self.cross_deg(float(s)) for s in self._s])
        self._mu = np.array([self.mu_max(float(s)) for s in self._s])
        self._r = np.array([self.roughness(float(s)) for s in self._s])
        self._z = np.concatenate([[0.0], np.cumsum(np.tan(np.deg2rad(self._g)) * self._ds)])[:-1]

    def _at(self, s: float, table: np.ndarray, default: float) -> float:
        return float(np.interp(s, self._s, table, left=float(table[0]), right=default))

    # ---- 基本查询 ----
    def grade(self, s: float) -> float:
        return self._at(s, self._g, self.grade_deg(float(self._s[-1])))

    def elevation(self, s: float) -> float:
        return float(np.interp(s, self._s, self._z))

    def cross_slope(self, s: float) -> float:
        return self._at(s, self._c, self.cross_deg(float(self._s[-1])))

    def friction(self, s: float) -> float:
        return self._at(s, self._mu, self.mu_max(float(self._s[-1])))

    def rough(self, s: float) -> float:
        return self._at(s, self._r, self.roughness(float(self._s[-1])))

    def curve(self, s: float) -> float:
        return float(self.curvature(float(s)))

    def soil(self, s: float) -> int:
        return int(round(float(self.soil_class(float(s)))))

    def height(self, s: float, y: float) -> float:
        """世界高程：纵向积分高程 + 横坡倾斜 + 粗糙度起伏。"""
        z = self.elevation(s)
        z += np.tan(np.deg2rad(self.cross_slope(s))) * y
        r = self.rough(s)
        # 两组不同波长的起伏，保证高程图平面拟合能还原出粗糙度统计量
        # 起伏波长取 1.1~1.6 m（大于高程图 0.15 m 分辨率的 7 倍），避免采样混叠
        z += r * (0.7 * np.sin(2 * np.pi * s / 1.6 + 1.3 * y) + 0.5 * np.sin(2 * np.pi * (s + 0.8 * y) / 1.1))
        return float(z)

    def sample_grid(
        self,
        s0: float,
        y0: float,
        rows: int,
        cols: int,
        res: float,
        noise_sigma: float = 0.0,
        rng: np.random.Generator | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """采样车头前方的高程图。

        返回 (elevation, s_axis, y_axis)，elevation[i, j] 对应 (s0 + i*res, y0 + (j - cols//2)*res)。
        """
        s_axis = s0 + np.arange(rows) * res
        y_axis = y0 + (np.arange(cols) - cols // 2) * res
        ss, yy = np.meshgrid(s_axis, y_axis, indexing="ij")
        elev = np.empty_like(ss)
        for i in range(rows):
            for j in range(cols):
                elev[i, j] = self.height(float(ss[i, j]), float(yy[i, j]))
        if noise_sigma > 0 and rng is not None:
            elev = elev + rng.normal(0.0, noise_sigma, size=elev.shape)
        return elev, s_axis, y_axis


# ---------------------------------------------------------------------------
# 预置场景地形
# ---------------------------------------------------------------------------


def soft_slope_profile(length: float = 18.0) -> TerrainProfile:
    """松软土长陡坡：8° 起步 → 20° 主坡 → 14° 平台，中途有松软带（μ≈0.64）。"""

    def grade(s: float) -> float:
        if s < 3.0:
            return 8.0
        if s < 5.0:
            return 8.0 + (s - 3.0) / 2.0 * 14.0
        if s < 13.0:
            return 20.0 + 0.8 * np.sin(s * 1.1)
        if s < 15.5:
            return 20.0 - (s - 13.0) / 2.5 * 6.0
        return 14.0 + 0.5 * np.sin(s)

    def mu(s: float) -> float:
        # 3.5~7.5 m 为雨后松软带，13 m 之后为压实土
        if 3.5 <= s < 7.5:
            return 0.64 - 0.02 * np.sin(s * 2.0)
        if s >= 13.0:
            return 0.72
        return 0.72

    def rough(s: float) -> float:
        return 0.024 + 0.012 * np.exp(-((s - 9.0) / 2.2) ** 2)

    def soil(s: float) -> int:
        return 2 if 3.5 <= s < 7.5 else 0

    return TerrainProfile(
        name="松软土长陡坡",
        length=length,
        grade_deg=grade,
        cross_deg=lambda s: 2.0 * np.sin(s / 4.0),
        mu_max=mu,
        roughness=rough,
        curvature=lambda s: 0.06 * np.sin(s / 3.0),
        soil_class=soil,
    )


def cross_slope_profile(length: float = 16.0) -> TerrainProfile:
    """横坡运输：14° 横坡 + 弯道，重载时侧倾风险最高。"""

    def cross(s: float) -> float:
        return 6.0 + 8.0 * np.clip((s - 2.0) / 4.0, 0.0, 1.0) * (1.0 if s < 11.0 else 0.8)

    return TerrainProfile(
        name="横坡弯道重载",
        length=length,
        grade_deg=lambda s: 6.0 + 2.0 * np.sin(s / 3.0) if s < 12.0 else 4.0,
        cross_deg=cross,
        mu_max=lambda s: 0.66,
        roughness=lambda s: 0.028,
        curvature=lambda s: 0.16 * np.exp(-((s - 8.0) / 3.0) ** 2),
        soil_class=lambda s: 1,
    )


def hill_hold_profile(length: float = 12.0) -> TerrainProfile:
    """坡道停车：18° 松软坡，用于考核驻车与起步。"""
    return TerrainProfile(
        name="坡道停车起步",
        length=length,
        grade_deg=lambda s: 18.0,
        cross_deg=lambda s: 3.0,
        mu_max=lambda s: 0.58,
        roughness=lambda s: 0.022,
        soil_class=lambda s: 2,
    )


def rough_downhill_profile(length: float = 20.0) -> TerrainProfile:
    """起伏路面长下坡：-16°，粗糙度 6~8 cm，考核下行限速与制动距离。"""

    def rough(s: float) -> float:
        return 0.055 + 0.02 * np.sin(s / 2.4)

    return TerrainProfile(
        name="起伏路面长下坡",
        length=length,
        grade_deg=lambda s: -16.0 if s > 2.0 else -6.0,
        cross_deg=lambda s: 4.0 * np.sin(s / 5.0),
        mu_max=lambda s: 0.58,
        roughness=rough,
        soil_class=lambda s: 1,
    )


def steep_wet_profile(length: float = 12.0) -> TerrainProfile:
    """超出能力的湿滑陡坡 26°：正确行为是识别并拒绝，而不是硬爬。

    前 4 m 为 6° 的缓坡接近段，保证系统有时间看到前方陡坡并安全停车。
    """

    def grade(s: float) -> float:
        if s < 4.0:
            return 6.0
        if s < 5.5:
            return 6.0 + (s - 4.0) / 1.5 * 20.0
        return 26.0

    return TerrainProfile(
        name="湿滑超限陡坡",
        length=length,
        grade_deg=grade,
        cross_deg=lambda s: 5.0,
        mu_max=lambda s: 0.50,
        roughness=lambda s: 0.03,
        soil_class=lambda s: 3,
    )


ALL_PROFILES: dict[str, Callable[[], TerrainProfile]] = {
    "soft_slope": soft_slope_profile,
    "cross_slope": cross_slope_profile,
    "hill_hold": hill_hold_profile,
    "rough_downhill": rough_downhill_profile,
    "steep_wet": steep_wet_profile,
}
