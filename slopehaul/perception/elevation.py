"""高程图几何特征：局部平面拟合 → 纵坡 / 横坡 / 粗糙度 / 台阶高度。

实现对应 Fast-LIO traversibility mapper 的做法：在滑动窗口内做最小二乘平面拟合，
法向量给出坡度与坡向，拟合残差 RMS 作为粗糙度，窗口内相邻栅格最大高差作为台阶高度，
最后按车体足迹做膨胀，得到可通行性栅格。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class TerrainFeatures:
    grade_deg: float          # 沿行进方向的纵坡
    cross_deg: float          # 横坡
    slope_deg: float          # 合成坡度（法向量倾角）
    roughness: float          # 拟合残差 RMS，m
    step_height: float        # 台阶高度，m
    grid: np.ndarray          # 每个栅格的坡度图
    roughness_map: np.ndarray
    cost_map: np.ndarray


def _design(window: int, res: float) -> np.ndarray:
    idx = np.arange(window) - window // 2
    xs, ys = np.meshgrid(idx * res, idx * res, indexing="ij")
    a = np.stack([np.ones(window * window), xs.ravel(), ys.ravel()], axis=1)
    return a


def terrain_features(
    elev: np.ndarray,
    res: float,
    window: int = 5,
    footprint_margin: float = 0.35,
) -> TerrainFeatures:
    rows, cols = elev.shape
    half = window // 2
    a = _design(window, res)
    pinv = np.linalg.pinv(a)

    slope_map = np.zeros((rows, cols))
    rough_map = np.zeros((rows, cols))
    step_map = np.zeros((rows, cols))
    grade_map = np.zeros((rows, cols))
    cross_map = np.zeros((rows, cols))

    for i in range(half, rows - half):
        for j in range(half, cols - half):
            patch = elev[i - half : i + half + 1, j - half : j + half + 1].ravel()
            coef = pinv @ patch
            gx, gy = coef[1], coef[2]            # 沿 s、横向 y 的梯度
            slope_map[i, j] = np.rad2deg(np.arctan(np.hypot(gx, gy)))
            grade_map[i, j] = np.rad2deg(np.arctan(gx))
            cross_map[i, j] = np.rad2deg(np.arctan(gy))
            residual = patch - a @ coef
            rough_map[i, j] = float(np.sqrt(np.mean(residual ** 2)))
            # 台阶：相邻栅格沿行进方向的最大高差（对长波起伏不敏感，只捕捉真正落差）
            block = elev[i - half : i + half + 1, j - half : j + half + 1]
            step_map[i, j] = float(np.max(np.abs(np.diff(block, axis=0))))

    # 足迹膨胀：取邻域内最不利值（前向 n_fwd 个栅格，横向 ±margin）
    n_fwd = max(1, int(round(0.55 / res)))
    n_lat = max(1, int(round(footprint_margin / res)))
    slope_buf = slope_map.copy()
    rough_buf = rough_map.copy()
    step_buf = step_map.copy()
    for di in range(n_fwd + 1):
        sl = np.roll(slope_map, -di, axis=0)
        ro = np.roll(rough_map, -di, axis=0)
        st = np.roll(step_map, -di, axis=0)
        for dj in range(-n_lat, n_lat + 1):
            slope_buf = np.maximum(slope_buf, np.roll(sl, dj, axis=1))
            rough_buf = np.maximum(rough_buf, np.roll(ro, dj, axis=1))
            step_buf = np.maximum(step_buf, np.roll(st, dj, axis=1))

    slope_ahead = float(np.max(slope_buf[half:, :]) if rows > half else slope_map.max())
    rough_ahead = float(np.max(rough_buf[half:, :]))
    step_ahead = float(np.max(step_buf[half:, :]))
    grade_now = float(np.median(grade_map[max(half, 3) : max(half, 3) + 2, :])) if rows > 5 else float(grade_map[half, cols // 2])
    cross_now = float(np.median(cross_map[max(half, 3) : max(half, 3) + 2, :])) if rows > 5 else 0.0
    slope_now = float(np.median(slope_map[max(half, 3) : max(half, 3) + 2, :])) if rows > 5 else slope_ahead

    return TerrainFeatures(
        grade_deg=grade_now,
        cross_deg=cross_now,
        slope_deg=slope_now,
        roughness=rough_ahead,
        step_height=step_ahead,
        grid=slope_buf,
        roughness_map=rough_buf,
        cost_map=step_buf,
    )
