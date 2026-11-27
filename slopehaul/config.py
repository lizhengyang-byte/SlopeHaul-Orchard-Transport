"""集中参数配置：车辆、轮胎-土壤、感知、控制。

所有默认值来自一台 4 轮独立驱动坡地重载运输平台（履带式与轮式均可运行的参数区间），
单位统一为 SI（m, kg, s, N, Nm, deg）。
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict


@dataclass
class VehicleParams:
    # ---- 几何与质量 ----
    mass_empty: float = 320.0          # 整备质量 kg
    payload_max: float = 200.0         # 额定载重 kg（果实筐 / 肥料 / 喷雾桶）
    wheelbase: float = 1.25            # 轴距 m
    track: float = 1.10                # 轮距 m
    wheel_radius: float = 0.24         # 车轮半径 m
    cg_height_empty: float = 0.52      # 空载质心高度 m
    cg_height_full: float = 0.60       # 满载质心高度 m
    cg_from_rear: float = 0.55         # 质心到后轴距离占轴距比例
    wheel_inertia: float = 3.2        # 单轮转动惯量 kg·m²

    # ---- 驱动能力 ----
    n_wheels: int = 4
    max_wheel_torque: float = 190.0     # 单轮峰值扭矩 Nm（约 1.5 kW @ 25 rad/s）
    max_wheel_speed: float = 7.5       # 单轮最高角速度 rad/s
    brake_torque: float = 120.0        # 单轮制动扭矩 Nm
    aux_power: float = 180.0           # 液压/电控等附件功率 W

    # ---- 轮胎与土壤 ----
    crr_hard: float = 0.030            # 硬土/水泥路面滚动阻力系数
    crr_soft: float = 0.095            # 松软土壤滚动阻力系数
    slip_opt: float = 0.18             # 峰值附着对应的滑移率
    slip_shape: float = 6.0            # 附着-滑移曲线陡度
    sinkage_k: float = 0.020           # 松软地面下沉量系数 m/kN
    static_gain: float = 1.12          # 静摩擦 / 峰值动摩擦

    # ---- 稳定性 ----
    roll_limit_deg: float = 32.0       # 允许侧倾角（含地形横坡）
    pitch_limit_deg: float = 34.0      # 允许俯仰角
    stability_margin: float = 0.75     # 稳定包络使用比例（0.75 表示留 25% 余量）

    # ---- 气动与其他 ----
    drag_coeff: float = 0.9            # 0.5*rho*Cd*A, N/(m/s)^2

    def mass(self, payload: float) -> float:
        return self.mass_empty + payload

    def cg_height(self, payload: float) -> float:
        ratio = min(max(payload / self.payload_max, 0.0), 1.0)
        return self.cg_height_empty + (self.cg_height_full - self.cg_height_empty) * ratio

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class PerceptionParams:
    grid_res: float = 0.15             # 高程图分辨率 m
    grid_rows: int = 34                # 车头方向 5.1 m
    grid_cols: int = 24                # 横向 3.6 m
    plane_window: int = 5              # 平面拟合窗口（奇数）
    depth_sigma: float = 0.012         # 深度噪声标准差 m
    pitch_noise: float = 0.25          # IMU 俯仰噪声 deg
    roll_noise: float = 0.25           # IMU 侧倾噪声 deg
    imu_bias: float = 0.6              # IMU 安装偏置 deg
    complementary_alpha: float = 0.82  # IMU 高频权重
    slope_block_deg: float = 27.0      # 绝对不可通行的坡度
    roughness_block: float = 0.105      # 不可通行粗糙度 m
    step_block: float = 0.16           # 不可通行台阶高度 m
    footprint_margin: float = 0.35     # 轮距方向膨胀 m（参照 traversability mapper）


@dataclass
class ControlParams:
    dt: float = 0.02
    speed_kp: float = 0.85
    speed_ki: float = 0.20
    slip_kp: float = 2.4
    slip_ki: float = 0.9
    slip_integral_limit: float = 0.6
    torque_rate_limit: float = 900.0   # Nm/s，抑制扭矩突变
    lookahead_min: float = 2.5         # 预瞄距离下限 m
    lookahead_max: float = 6.0
    max_speed_flat: float = 1.35       # 平路最高作业速度 m/s
    max_speed_slope: float = 0.85
    speed_filter_tau: float = 0.45     # 速度指令一阶滤波 s
    stop_decel_reserve: float = 0.35   # 制动减速度储备比例
    hill_hold_brake_ratio: float = 0.85
    safety_roll_trigger: float = 0.80  # 达到稳定包络比例触发降速
    safety_pitch_trigger: float = 0.85
    min_operating_speed: float = 0.05


@dataclass
class ScenarioParams:
    name: str = "demo"
    length: float = 18.0
    payload: float = 150.0
    initial_speed: float = 0.0
    duration: float = 40.0
    stop_at: float | None = None       # 在 s 处执行坡道停车
    restart_after: float | None = None  # 停车后重新起步的时间 s
    dt: float = 0.02


@dataclass
class Config:
    vehicle: VehicleParams = field(default_factory=VehicleParams)
    perception: PerceptionParams = field(default_factory=PerceptionParams)
    control: ControlParams = field(default_factory=ControlParams)
    seed: int = 20261127


DEFAULT_CONFIG = Config()
