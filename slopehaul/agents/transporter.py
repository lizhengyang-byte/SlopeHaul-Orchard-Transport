"""运输平台智能体：感知 → 估计 → 规划 → 控制 → 安全 的完整闭环。

该类同时提供两种策略，便于同一地形上做 A/B 对照：
- `proposed`：地形可通行性感知 + 滑移率闭环 + 稳定包络 + 坡道驻车；
- `baseline`：固定速度指令、四轮均分扭矩、无滑移控制、固定制动、无地形判断。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..config import Config
from ..control.adaptive import AdaptiveController
from ..control.hillhold import HillHoldController
from ..control.safety import SafetySupervisor
from ..control.stability import StabilitySupervisor
from ..estimation.load import LoadEstimator
from ..estimation.slip import SlipEstimator
from ..perception.elevation import terrain_features
from ..perception.fusion import ComplementaryPose
from ..perception.soil_classifier import SoilClassifier, classify_soil
from ..perception.traversability import TraversabilityResult, evaluate
from ..sim.sensors import SensorFrame
from ..sim.vehicle import VehicleModel, VehicleState


@dataclass
class StepDiagnostics:
    traversability: TraversabilityResult
    stability: object
    safety: object
    hold: object
    soil_name: str
    v_est: float
    slip_mean: float
    mu_est: float
    torque: np.ndarray
    brake: float
    mode: str
    notes: list[str] = field(default_factory=list)


class TransporterAgent:
    def __init__(
        self,
        cfg: Config,
        model: VehicleModel,
        policy: str = "proposed",
        soil_model_path: str | None = None,
    ) -> None:
        self.cfg = cfg
        self.vp = cfg.vehicle
        self.pp = cfg.perception
        self.cp = cfg.control
        self.model = model
        self.policy = policy

        self.pose = ComplementaryPose(self.pp.complementary_alpha)
        self.slip = SlipEstimator(self.vp)
        self.load = LoadEstimator(self.vp)
        self.adaptive = AdaptiveController(self.vp, self.cp)
        self.stability = StabilitySupervisor(self.vp, self.cp)
        self.hold = HillHoldController(self.vp, self.cp)
        self.safety = SafetySupervisor(self.cp)
        self.classifier = SoilClassifier(soil_model_path)

        self.stop_timer = 0.0
        self.mode = "drive"
        self.reverse_timer = 0.0
        self.last_diag: StepDiagnostics | None = None
        self.last_normals = np.full(self.vp.n_wheels, self.vp.mass(150.0) * 9.81 / self.vp.n_wheels)
        self._block_streak = 0

    # ------------------------------------------------------------------
    def reset(self, speed: float = 0.0) -> None:
        self.pose = ComplementaryPose(self.pp.complementary_alpha)
        self.slip = SlipEstimator(self.vp)
        self.slip.ekf.reset(max(speed, 0.0))
        self.load = LoadEstimator(self.vp)
        self.adaptive.reset(speed)
        self.hold.reset()
        self.safety.reset()
        self.stop_timer = 0.0
        self.mode = "drive"
        self.reverse_timer = 0.0

    # ------------------------------------------------------------------
    def step(
        self,
        frame: SensorFrame,
        state: VehicleState,
        info: dict,
        dt: float,
        stop_request: bool,
        speed_target_override: float | None = None,
    ) -> tuple[np.ndarray, float, bool, StepDiagnostics]:
        vp = self.vp

        # ---- 1. 感知 ----
        feats = terrain_features(frame.elevation, self.pp.grid_res, self.pp.plane_window, self.pp.footprint_margin)
        soil = classify_soil(self.classifier, frame.appearance, feats.slope_deg, feats.roughness)

        load_est = self.load.update(dt, frame.load_meas, info["grade"], info["a_long"])

        # ---- 2. 姿态估计 ----
        pose = self.pose.update(
            dt,
            frame.pitch_meas,
            frame.roll_meas,
            frame.pitch_rate,
            frame.yaw_rate,
            feats.grade_deg,
            feats.cross_deg,
            perception_trust=1.0 - min(feats.roughness / 0.12, 1.0),
        )

        # ---- 3. 速度 / 滑移 / 附着估计 ----
        speed_est = self.slip.update(
            dt,
            frame.wheel_speed,
            frame.accel_forward,
            frame.ground_speed_meas,
            np.maximum(self._last_normals(), 1.0),
            abs(info["f_drive"]),
            info["grade"],
            load_est.mass,
        )
        mu_est = self.slip.rls.mu_max

        # ---- 4. 可通行性 ----
        lookahead = max(
            self.cp.lookahead_min,
            min(self.cp.lookahead_max, self.cp.lookahead_min + max(speed_est.v, 0.0) * 2.5),
        )
        trav = evaluate(
            feats,
            soil,
            self.pp,
            self.cp,
            mu_online=mu_est,
            speed=speed_est.v,
            load=load_est.payload,
            curvature=info.get("curvature", 0.0),
            lookahead=lookahead,
        )

        # ---- 5. 稳定性 ----
        stab = self.stability.evaluate(pose, load_est, speed_est.v, info.get("curvature", 0.0), feats.cross_deg)

        # ---- 6. 安全监督 ----
        v_cmd_preview = trav.v_limit if not trav.blocked else 0.0
        if speed_target_override is not None:
            v_cmd_preview = speed_target_override
        safe = self.safety.evaluate(
            dt,
            speed_est.v,
            v_cmd_preview,
            stab,
            info.get("torque_util", 0.0),
            frame.motor_temp,
            pose.pitch,
            pose.roll,
            trav.blocked,
            info["grade"],
        )

        # 不可通行判定做 3 帧确认，抑制单帧坡度噪声造成的频繁启停
        self._block_streak = self._block_streak + 1 if trav.blocked else 0
        block_active = self._block_streak >= 3

        # ---- 7. 策略分支 ----
        if self.policy == "baseline":
            torque, brake, parked, notes = self._baseline_act(frame, state, info, speed_est.v, safe)
            diag = StepDiagnostics(
                traversability=trav,
                stability=stab,
                safety=safe,
                hold=self.hold.update(dt, state.s, speed_est.v, info["grade"], 0.75 if safe.allow else 0.9,
                                      float(np.sum(self._last_normals())), load_est.mass, info["crr"], stop_request),
                soil_name=soil.soil_name,
                v_est=speed_est.v,
                slip_mean=speed_est.slip_ratio_mean,
                mu_est=mu_est,
                torque=torque,
                brake=brake,
                mode="baseline",
                notes=notes,
            )
            self.last_diag = diag
            return torque, brake, parked, diag

        # 安全层要求回退
        if safe.reverse and self.mode != "reverse":
            self.mode = "reverse"
            self.reverse_timer = 0.0
        if self.mode == "reverse":
            self.reverse_timer += dt
            if self.reverse_timer > 2.2:
                self.mode = "drive"
                self.reverse_timer = 0.0

        hold = self.hold.update(
            dt,
            state.s,
            speed_est.v,
            info["grade"],
            mu_est,
            float(np.sum(self._last_normals())),
            load_est.mass,
            info["crr"],
            stop_request or (safe.emergency_stop and self.mode == "drive"),
        )
        parked = hold.state in ("HOLD", "BRAKING")
        if hold.slipping:
            self.mode = "recover"

        cmd = self.adaptive.act(
            dt,
            speed_est.v,
            speed_est,
            feats,
            trav,
            pose,
            load_est,
            np.maximum(self._last_normals(), 1.0),
            info["grade"],
            info["crr"],
            safe.scale,
            self.mode if self.mode in ("reverse", "drive") else "drive",
            mu_est,
        )
        if speed_target_override is not None and self.mode == "drive":
            # 用于“定速对照”实验：保留感知与稳定层，仅替换速度目标
            cmd.v_target = float(speed_target_override)

        brake = max(cmd.brake, hold.brake if parked else 0.0)
        if block_active:
            brake = max(brake, 0.6)
        if safe.emergency_stop:
            torque = np.zeros(vp.n_wheels)
            brake = 1.0
        else:
            torque = cmd.torque

        notes = list(cmd.notes)
        if safe.code != "OK":
            notes.append(f"[{safe.code}] {safe.message}")
        if hold.message:
            notes.append(hold.message)
        if self.mode == "reverse":
            notes.append("卡滞回退中")

        diag = StepDiagnostics(
            traversability=trav,
            stability=stab,
            safety=safe,
            hold=hold,
            soil_name=soil.soil_name,
            v_est=speed_est.v,
            slip_mean=speed_est.slip_ratio_mean,
            mu_est=mu_est,
            torque=torque,
            brake=brake,
            mode=self.mode,
            notes=notes,
        )
        self.last_diag = diag
        return torque, brake, parked, diag

    # ------------------------------------------------------------------
    def _baseline_act(self, frame, state, info, v, safe) -> tuple[np.ndarray, float, bool, list[str]]:
        """基线：定速 0.9 m/s、均分扭矩、无滑移闭环、固定制动。"""
        mass = self.vp.mass(state.load)
        theta = np.deg2rad(info["grade"])
        v_target = 0.9 if not safe.emergency_stop else 0.0
        err = v_target - v
        force = mass * 9.81 * np.sin(theta) + 0.05 * mass * 9.81 + mass * 0.6 * err
        if v_target <= 0.0:
            force = 0.0
        torque_wheel = force * self.vp.wheel_radius / self.vp.n_wheels
        torque = np.full(self.vp.n_wheels, np.clip(torque_wheel, -self.vp.max_wheel_torque, self.vp.max_wheel_torque))
        brake = 0.45 if (safe.emergency_stop or v_target <= 0.0) else 0.0
        notes = ["基线策略：定速 0.9 m/s，均分扭矩"]
        return torque, brake, brake > 0, notes

    # ------------------------------------------------------------------
    def _last_normals(self) -> np.ndarray:
        return np.maximum(self.last_normals, 1.0)

    def set_normals(self, loads: np.ndarray) -> None:
        self.last_normals = np.maximum(np.asarray(loads, dtype=float), 1.0)
