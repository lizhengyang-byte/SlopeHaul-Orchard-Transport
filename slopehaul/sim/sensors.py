"""传感器仿真：深度相机高程图、IMU、轮速编码器、GNSS/视觉里程计、载重传感器。

这里刻意保留噪声、偏置和量化误差，因为本项目的控制策略必须对不完美感知鲁棒，
否则论文里的“自适应”就只是在 ground truth 上作弊。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import PerceptionParams, VehicleParams
from .terrain import TerrainProfile


@dataclass
class SensorFrame:
    t: float
    s: float
    y: float
    elevation: np.ndarray
    s_axis: np.ndarray
    y_axis: np.ndarray
    appearance: np.ndarray
    pitch_meas: float
    roll_meas: float
    pitch_rate: float
    yaw_rate: float
    accel_forward: float
    accel_lateral: float
    wheel_speed: np.ndarray
    ground_speed_meas: float
    load_meas: float
    motor_temp: float


class SensorSuite:
    """把仿真真值转换成带噪声的传感器读数。"""

    def __init__(
        self,
        pp: PerceptionParams,
        vp: VehicleParams,
        seed: int = 20261127,
        sensor_dropout: bool = False,
    ) -> None:
        self.pp = pp
        self.vp = vp
        self.rng = np.random.default_rng(seed)
        self.sensor_dropout = sensor_dropout
        self._imu_bias_pitch = self.rng.normal(0.0, 0.3)
        self._imu_bias_roll = self.rng.normal(0.0, 0.3)
        self._gyro_bias = self.rng.normal(0.0, 0.02, size=2)

    # ------------------------------------------------------------------
    def read(
        self,
        t: float,
        s: float,
        y: float,
        terrain: TerrainProfile,
        pitch_true: float,
        roll_true: float,
        yaw_rate: float,
        accel_forward: float,
        accel_lateral: float,
        wheel_omega: np.ndarray,
        ground_speed: float,
        load: float,
    ) -> SensorFrame:
        rng = self.rng
        elev, sa, ya = terrain.sample_grid(
            s,
            y,
            self.pp.grid_rows,
            self.pp.grid_cols,
            self.pp.grid_res,
            noise_sigma=self.pp.depth_sigma,
            rng=rng,
        )
        if self.sensor_dropout:
            # 强光/扬尘导致 8% 的深度点缺失，用邻域均值填补
            mask = rng.random(elev.shape) < 0.08
            elev[mask] = np.nan
            nan_mask = np.isnan(elev)
            if nan_mask.any():
                elev[nan_mask] = np.nanmedian(elev)

        appearance = self._appearance(terrain.soil(s))
        pitch_meas = pitch_true + self._imu_bias_pitch + rng.normal(0, self.pp.pitch_noise)
        roll_meas = roll_true + self._imu_bias_roll + rng.normal(0, self.pp.roll_noise)
        pitch_rate = rng.normal(0, 0.6)
        wheel_speed = wheel_omega + rng.normal(0, 0.03, size=wheel_omega.shape)
        # 视觉/卫星里程计：打滑时误差会显著增大（模拟视觉退化）
        slip_proxy = abs(wheel_omega.mean() * self.vp.wheel_radius - ground_speed)
        ground_meas = ground_speed + rng.normal(0, 0.03 + 0.15 * min(slip_proxy, 1.0))
        load_meas = load + rng.normal(0, 6.0)
        motor_temp = 42.0 + 0.02 * np.abs(wheel_omega).sum()

        return SensorFrame(
            t=t,
            s=s,
            y=y,
            elevation=elev,
            s_axis=sa,
            y_axis=ya,
            appearance=appearance,
            pitch_meas=pitch_meas,
            roll_meas=roll_meas,
            pitch_rate=pitch_rate,
            yaw_rate=yaw_rate,
            accel_forward=accel_forward,
            accel_lateral=accel_lateral,
            wheel_speed=wheel_speed,
            ground_speed_meas=ground_meas,
            load_meas=load_meas,
            motor_temp=motor_temp,
        )

    # ------------------------------------------------------------------
    def _appearance(self, soil_class: int) -> np.ndarray:
        """32×32 单通道地表纹理图，用于土壤类型分类。

        真实系统对应相机 RGB 裁剪；这里用统计特征可复现的合成纹理替代，
        并在文档中明确其为代理数据（proxy），不宣称真实田间精度。
        """
        profile = {
            0: (0.62, 0.035),   # 压实土：亮、纹理细
            1: (0.46, 0.105),    # 草地：中等亮度、纹理粗
            2: (0.53, 0.075),    # 松软土：偏亮、颗粒感
            3: (0.30, 0.020),    # 湿滑黏土：暗、平滑（有反光）
        }
        base, tex = profile.get(soil_class, (0.5, 0.05))
        rng = self.rng
        field = rng.normal(0.0, 1.0, size=(36, 36))
        # 简易低通，形成颗粒尺度纹理
        for _ in range(2):
            field = 0.25 * (
                np.roll(field, 1, 0) + np.roll(field, -1, 0) + np.roll(field, 1, 1) + np.roll(field, -1, 1)
            ) + 0.5 * field
        img = base + tex * field[2:-2, 2:-2]
        if soil_class == 3:
            img += 0.05 * np.exp(-((np.arange(32)[:, None] - 16) ** 2 + (np.arange(32)[None, :] - 12) ** 2) / 60.0)
        img += rng.normal(0, 0.012, size=img.shape)
        return np.clip(img, 0.0, 1.0)
