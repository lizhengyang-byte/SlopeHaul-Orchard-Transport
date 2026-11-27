"""车辆动力学：四轮独立驱动、滑移率相关附着、载荷转移、侧倾/俯仰与驻车。

模型定位是“准静态车体 + 轮级滑移动力学”：车体用纵向单轨模型，车轮保留各自的
转动惯量与滑移率，因此可以复现负重上坡时驱动轮空转、附着系数跌落、车体后溜等
典型失效过程。横向只保留准静态侧倾与载荷转移，适合 1 m/s 级坡地运输平台。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import VehicleParams
from .terrain import TerrainProfile


@dataclass
class VehicleState:
    s: float = 0.0
    y: float = 0.0
    v: float = 0.0
    omega: np.ndarray = None          # 4 轮角速度 rad/s
    roll: float = 0.0                  # 侧倾角 deg，右倾为正
    pitch: float = 0.0                 # 俯仰角 deg，抬头为正
    yaw_rate: float = 0.0
    load: float = 150.0
    distance: float = 0.0
    energy: float = 0.0
    tipped: bool = False
    stuck_time: float = 0.0
    mode: str = "drive"

    def __post_init__(self) -> None:
        if self.omega is None:
            self.omega = np.zeros(4)


@dataclass
class WheelForce:
    normal: np.ndarray
    traction: np.ndarray
    slip: np.ndarray
    mu: np.ndarray
    sinkage: np.ndarray


class VehicleModel:
    """车辆 + 轮胎-土壤相互作用。"""

    def __init__(self, vp: VehicleParams) -> None:
        self.vp = vp
        self.slip_state = np.zeros(vp.n_wheels)
        self.tire_tau = 0.06

    # ------------------------------------------------------------------
    def mu_curve(self, slip: np.ndarray, mu_max: np.ndarray) -> np.ndarray:
        """可用附着曲线：静摩擦 → 峰值（s≈slip_opt）→ 滑移后衰减。

        s=0 时为静摩擦上限 static_gain·μ_max；到 slip_opt 时降到 μ_max（峰值）；
        超过峰值后按线性衰减到 0.55·μ_max（s=1）。这样既能表示“扭矩未超静摩擦时不打滑”，
        也能表示“轮子空转后附着跌落”，且曲线在 0 附近连续，不会出现数值悬崖。
        """
        s = np.clip(np.abs(slip), 0.0, 1.0)
        s_opt = self.vp.slip_opt
        gain = self.vp.static_gain
        rising = mu_max * (gain - (gain - 1.0) * (s / s_opt))
        falling = mu_max * (1.0 - 0.45 * (s - s_opt) / (1.0 - s_opt))
        return np.where(s <= s_opt, rising, falling)

    # ------------------------------------------------------------------
    def normal_loads(
        self,
        grade_deg: float,
        cross_deg: float,
        a_lat: float,
        load: float,
        softness: float,
    ) -> np.ndarray:
        vp = self.vp
        m = vp.mass(load)
        g = 9.81
        theta = np.deg2rad(grade_deg)
        h = vp.cg_height(load)
        L = vp.wheelbase
        lf = vp.cg_from_rear * L      # 后轴到质心
        lr = L - lf                    # 质心到前轴

        n_front = m * g * np.cos(theta) * lr / L - m * g * h * np.sin(theta) / L
        n_rear = m * g * np.cos(theta) * lf / L + m * g * h * np.sin(theta) / L
        n_front = max(n_front, 0.05 * m * g)
        n_rear = max(n_rear, 0.05 * m * g)

        dn = m * a_lat * h / vp.track
        loads = np.array([
            max(n_front / 2 - dn / 2, 0.0),
            max(n_front / 2 + dn / 2, 0.0),
            max(n_rear / 2 - dn / 2, 0.0),
            max(n_rear / 2 + dn / 2, 0.0),
        ])
        # 松软地面接地压力下降，等效承载减小
        loads *= 1.0 - 0.10 * softness
        return loads

    # ------------------------------------------------------------------
    def step(
        self,
        state: VehicleState,
        terrain: TerrainProfile,
        torque_cmd: np.ndarray,
        brake_cmd: float,
        a_lat: float,
        softness: float,
        dt: float,
        parked: bool = False,
    ) -> tuple[VehicleState, WheelForce, dict[str, float]]:
        vp = self.vp
        grade = terrain.grade(state.s)
        cross = terrain.cross_slope(state.s)
        mu_nominal = terrain.friction(state.s)
        m = vp.mass(state.load)
        g = 9.81
        theta = np.deg2rad(grade)

        loads = self.normal_loads(grade, cross, a_lat, state.load, softness)
        sinkage = vp.sinkage_k * loads / 1000.0
        mu_eff = mu_nominal * (1.0 - 0.18 * np.clip(sinkage / 0.03, 0.0, 1.0))

        # ---- 车轮-地面相互作用（滑移平衡模型） ----
        # 低速坡地运输平台的轮系时间常数（~0.1 s）远小于车体运动时间常数，
        # 因此不做轮速刚性积分，而用滑移率平衡方程：需求力超过静摩擦即滑移上升、
        # 附着跌落；需求力回落则滑移按 0.25 s 时间常数恢复。数值稳定且可复现空转过程。
        torque_cmd = np.clip(torque_cmd, -vp.max_wheel_torque, vp.max_wheel_torque)
        demanded = torque_cmd / vp.wheel_radius
        cap_static = vp.static_gain * mu_eff * loads

        ratio = np.abs(demanded) / np.maximum(cap_static, 1.0)
        grow = np.where(ratio > 1.0, (ratio - 1.0) * 2.5, -self.slip_state / 0.25)
        self.slip_state = np.clip(self.slip_state + grow * dt, 0.0, 1.0)
        slip = self.slip_state.copy()

        mu = self.mu_curve(slip, mu_eff)
        cap_avail = mu * loads
        traction = np.sign(demanded) * np.minimum(np.abs(demanded), cap_avail)

        # 轮速仅用于观测/记录：按滑移率由车速反算
        state.omega = state.v / vp.wheel_radius / np.maximum(1.0 - slip, 0.05)
        omega = state.omega

        # ---- 车体纵向 ----
        f_drive = float(np.sum(traction))
        f_grade = m * g * np.sin(theta)
        crr = vp.crr_hard + (vp.crr_soft - vp.crr_hard) * softness
        f_roll = crr * m * g * np.cos(theta)
        f_drag = vp.drag_coeff * state.v * abs(state.v)
        f_brake_mag = min(brake_cmd * vp.brake_torque * vp.n_wheels / vp.wheel_radius, m * 25.0)
        # 制动力始终与运动方向相反，且不足以反向驱动车辆
        f_brake = np.sign(state.v) * f_brake_mag if abs(state.v) > 0.02 else 0.0

        brake_tau = brake_cmd * vp.brake_torque

        if parked:
            # 驻车：只有静摩擦/制动能力足够才停得住，否则后溜
            cap_brake = brake_tau * vp.n_wheels / vp.wheel_radius
            cap_friction = vp.static_gain * float(np.sum(mu_eff * loads))
            hold_cap = min(cap_brake, cap_friction)
            need = f_grade - f_roll
            if hold_cap >= need:
                a = 0.0
                state.v = 0.0
            else:
                a = (need - hold_cap) / m * -1.0
                state.v = state.v + a * dt
            f_drive = 0.0
        else:
            a = (f_drive - f_grade - f_roll - f_drag - f_brake) / m
            state.v = state.v + a * dt

        if state.v < 0.0:
            state.v = max(state.v, -1.5)
        # 制动导致换向时收敛到零，避免“刹过头”变成倒车
        if brake_cmd > 0 and abs(state.v) < 0.03:
            state.v = 0.0

        state.s = state.s + state.v * dt
        state.distance = state.distance + abs(state.v) * dt

        # 打滑导致的无效位移（车轮转但不前进）
        wheel_dist = float(np.mean(np.abs(omega))) * vp.wheel_radius * dt
        slip_ratio = max(0.0, 1.0 - abs(state.v) * dt / max(wheel_dist, 1e-6))

        # ---- 姿态 ----
        roll_ss = cross + self._dynamic_roll(a_lat, state.load)
        pitch_ss = grade + self._dynamic_pitch(a, state.load)
        tau_att = 0.22
        state.roll += (roll_ss - state.roll) * min(dt / tau_att, 1.0)
        state.pitch += (pitch_ss - state.pitch) * min(dt / tau_att, 1.0)

        roll_cap = np.rad2deg(np.arctan(vp.track / (2.0 * vp.cg_height(state.load))) )
        if abs(state.roll) > roll_cap:
            state.tipped = True
        if abs(state.pitch) > 55.0:
            state.tipped = True

        # ---- 能量 ----
        p_mech = float(np.sum(np.abs(torque_cmd * omega))) + vp.aux_power
        state.energy += p_mech * dt

        # ---- 卡滞判定 ----
        if state.v < 0.08 and not parked:
            state.stuck_time += dt
        else:
            state.stuck_time = 0.0

        state.yaw_rate = state.v * terrain.curve(state.s)
        forces = WheelForce(normal=loads, traction=traction, slip=slip, mu=mu, sinkage=sinkage)
        info = {
            "grade": grade,
            "cross": cross,
            "mu_nominal": mu_nominal,
            "mu_eff": float(np.mean(mu_eff)),
            "f_drive": f_drive,
            "f_grade": f_grade,
            "f_roll": f_roll,
            "f_brake": f_brake,
            "slip_ratio": slip_ratio,
            "a_long": a,
            "roll_cap": roll_cap,
            "crr": crr,
        }
        return state, forces, info

    # ------------------------------------------------------------------
    def _dynamic_roll(self, a_lat: float, load: float) -> float:
        vp = self.vp
        h = vp.cg_height(load)
        phi = np.rad2deg(np.arctan(a_lat * h / (9.81 * vp.track / 2.0)))
        return float(0.55 * phi)

    def _dynamic_pitch(self, a_long: float, load: float) -> float:
        vp = self.vp
        h = vp.cg_height(load)
        return float(np.rad2deg(np.arctan(a_long * h / (9.81 * vp.wheelbase)) * 0.5))
