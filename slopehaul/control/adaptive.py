"""自适应纵向控制：限速规划 + 坡度前馈 + 滑移率闭环 + 载荷自适应扭矩分配。

与常规“定速 + 均分扭矩”的差别有三点：
1. 速度目标由地形感知的可通行性给出，而不是固定值（坡度越陡、越松软越慢）；
2. 前馈项显式包含坡度与滚阻，避免纯反馈在上坡时长期饱和；
3. 每个轮子独立闭环滑移率，按法向载荷分配扭矩，把“轮子空转”压回峰值附着点附近。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..config import ControlParams, VehicleParams


@dataclass
class ControlCommand:
    torque: np.ndarray
    brake: float
    v_target: float
    slip_target: float
    feedforward: float
    feedback: float
    mode: str
    torque_utilization: float
    notes: list[str] = field(default_factory=list)


class SpeedPlanner:
    """把可通行性限速、风险分和作业模式合成为平滑速度指令。"""

    def __init__(self, cp: ControlParams) -> None:
        self.cp = cp
        self.v_cmd = 0.0

    def reset(self, v: float = 0.0) -> None:
        self.v_cmd = v

    def plan(self, dt: float, v_limit: float, risk: float, blocked: bool, grade: float, mode: str) -> float:
        if blocked:
            target = 0.0
        elif mode == "reverse":
            target = -min(0.45, max(v_limit, 0.15) * 0.5)
        else:
            base = min(v_limit, self.cp.max_speed_flat)
            if abs(grade) > 12.0:
                base = min(base, self.cp.max_speed_slope)
            # 风险分连续降速：risk=1 时降到 35%
            target = base * (1.0 - 0.65 * min(risk, 1.0))
            target = max(target, 0.0)
        alpha = min(dt / self.cp.speed_filter_tau, 1.0)
        self.v_cmd += (target - self.v_cmd) * alpha
        return float(self.v_cmd)


class SlipController:
    """轮级滑移率 PI，带抗饱和与增益调度。"""

    def __init__(self, cp: ControlParams, vp: VehicleParams) -> None:
        self.cp = cp
        self.vp = vp
        self.integral = np.zeros(vp.n_wheels)
        self.slip_target = vp.slip_opt

    def reset(self) -> None:
        self.integral[:] = 0.0

    def correct(self, dt: float, slip_meas: np.ndarray, mu_est: float, load_ratio: np.ndarray) -> np.ndarray:
        # 滑移目标略低于峰值，留出 10% 附着裕度
        self.slip_target = float(np.clip(self.vp.slip_opt * 0.92, 0.08, 0.30))
        err = self.slip_target - slip_meas
        # 低附着时降低积分增益，防止积分饱和后轮子持续空转
        ki = self.cp.slip_ki * (0.4 + 0.6 * min(mu_est / 0.6, 1.0))
        self.integral = self.integral + ki * err * dt
        self.integral = np.clip(self.integral, -self.cp.slip_integral_limit, self.cp.slip_integral_limit)
        corr = self.cp.slip_kp * err + self.integral
        # 载荷越小的轮子越容易空转，修正量按载荷比例再放大一点
        corr = corr * (1.0 + 0.25 * (1.0 - load_ratio))
        # 只做“削减”不做“加码”：滑移低于目标时不应继续加大扭矩，否则会主动制造空转
        return np.clip(corr, -1.0, 0.0)


class AdaptiveController:
    def __init__(self, vp: VehicleParams, cp: ControlParams) -> None:
        self.vp = vp
        self.cp = cp
        self.planner = SpeedPlanner(cp)
        self.slip_ctrl = SlipController(cp, vp)
        self.speed_integral = 0.0
        self.prev_torque = np.zeros(vp.n_wheels)

    def reset(self, v: float = 0.0) -> None:
        self.planner.reset(v)
        self.slip_ctrl.reset()
        self.speed_integral = 0.0
        self.prev_torque[:] = 0.0

    # ------------------------------------------------------------------
    def act(
        self,
        dt: float,
        speed: float,
        speed_est,
        feats,
        trav,
        pose,
        load_est,
        normal_loads: np.ndarray,
        grade: float,
        crr: float,
        stability_scale: float,
        mode: str,
        mu_est: float,
    ) -> ControlCommand:
        v_target = self.planner.plan(dt, trav.v_limit, trav.risk, trav.blocked, grade, mode) * stability_scale

        mass = load_est.mass
        theta = np.deg2rad(grade)
        # 前馈：坡道分量 + 滚阻
        # 坡道分量给 15% 补偿系数，覆盖未建模的土壤压实/轮辙阻力
        grade_gain = 1.15 if grade > 6.0 else 1.0
        ff_force = mass * 9.81 * np.sin(theta) * grade_gain + crr * mass * 9.81 * np.cos(theta)
        ff_force = max(ff_force, -mass * 9.81 * 0.5)

        err = float(np.clip(v_target - speed, -0.8, 1.5))
        # 抗积分饱和：大误差（起步/超速）与饱和工况下停止积分
        if abs(err) < 0.6:
            ki = self.cp.speed_ki * (0.5 + 0.5 * min(max(load_est.confidence, 0.2), 1.0))
            self.speed_integral = self.speed_integral + ki * err * dt * mass
        self.speed_integral = float(np.clip(self.speed_integral, -260.0, 260.0))
        fb_force = mass * self.cp.speed_kp * err + self.speed_integral
        # 超速时允许回馈制动（负扭矩），而不是继续加速
        if err < -0.25:
            fb_force = max(fb_force, -0.35 * mass * 9.81)
        # 后溜保护：坡上出现负车速时不允许反馈项“帮倒忙”，并追加静态保持力
        if speed < -0.02 and v_target > 0.0:
            fb_force = max(fb_force, 0.18 * mass * 9.81)

        total_force = ff_force + fb_force
        if mode == "reverse":
            total_force = min(total_force, 0.0)
        if v_target <= 0.01 and speed < 0.25:
            total_force = min(total_force, 0.0)

        load_sum = max(float(np.sum(normal_loads)), 1.0)
        load_ratio = normal_loads / load_sum
        # 扭矩按法向载荷分配（附着上限与 N 成正比）
        wheel_force = total_force * load_ratio
        torque = wheel_force * self.vp.wheel_radius

        slip_corr = self.slip_ctrl.correct(dt, speed_est.slip, mu_est, load_ratio)
        torque = torque * (1.0 + slip_corr)
        # 深度空转保护：滑移率过高时直接削减驱动扭矩，给车轮“找回附着”的机会
        if float(np.mean(speed_est.slip)) > 0.55:
            torque = torque * 0.35

        # 扭矩变化率限制
        max_step = self.cp.torque_rate_limit * dt
        torque = np.clip(torque, self.prev_torque - max_step, self.prev_torque + max_step)
        torque = np.clip(torque, -self.vp.max_wheel_torque, self.vp.max_wheel_torque)
        self.prev_torque = torque.copy()

        utilization = float(np.max(np.abs(torque)) / self.vp.max_wheel_torque)
        notes = []
        if utilization > 0.9:
            notes.append("扭矩接近饱和：已优先保滑移率而非速度")
        if trav.blocked:
            notes.append(trav.block_reason)

        return ControlCommand(
            torque=torque,
            brake=0.0,
            v_target=v_target,
            slip_target=self.slip_ctrl.slip_target,
            feedforward=ff_force,
            feedback=fb_force,
            mode=mode,
            torque_utilization=utilization,
            notes=notes,
        )
