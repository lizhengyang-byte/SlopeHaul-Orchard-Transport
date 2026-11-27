"""仿真世界：把地形、车辆、传感器与智能体接成闭环，并记录全过程遥测。

同一地形上可切换 `proposed` / `baseline` 两种策略，随机种子固定，保证 A/B 可比。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..agents.transporter import TransporterAgent
from ..config import Config, ScenarioParams
from .sensors import SensorSuite
from .terrain import TerrainProfile
from .vehicle import VehicleModel, VehicleState


@dataclass
class RunResult:
    scenario: str
    policy: str
    terrain: str
    rows: list[dict] = field(default_factory=list)
    summary: dict = field(default_factory=dict)


def softness_of(terrain: TerrainProfile, s: float) -> float:
    cls = terrain.soil(s)
    return 0.0 if cls == 0 else (0.55 if cls == 1 else (1.0 if cls == 2 else 0.9))


def base_crr(terrain: TerrainProfile, s: float) -> float:
    return 0.030 + (0.095 - 0.030) * softness_of(terrain, s)


class Simulation:
    def __init__(
        self,
        cfg: Config,
        terrain: TerrainProfile,
        policy: str = "proposed",
        payload: float = 150.0,
        seed: int | None = None,
        sensor_dropout: bool = False,
        soil_model_path: str | None = None,
    ) -> None:
        self.cfg = cfg
        self.terrain = terrain
        self.policy = policy
        self.payload = payload
        self.seed = seed if seed is not None else cfg.seed
        self.model = VehicleModel(cfg.vehicle)
        self.sensors = SensorSuite(cfg.perception, cfg.vehicle, seed=self.seed, sensor_dropout=sensor_dropout)
        self.agent = TransporterAgent(cfg, self.model, policy=policy, soil_model_path=soil_model_path)

    # ------------------------------------------------------------------
    def run(self, scen: ScenarioParams) -> RunResult:
        cfg = self.cfg
        vp = cfg.vehicle
        terrain = self.terrain
        dt = cfg.control.dt

        state = VehicleState(s=0.0, y=0.0, v=scen.initial_speed, load=scen.payload)
        state.omega = np.full(vp.n_wheels, scen.initial_speed / vp.wheel_radius)
        self.agent.reset(scen.initial_speed)

        grade0 = terrain.grade(0.0)
        cross0 = terrain.cross_slope(0.0)
        normals = self.model.normal_loads(grade0, cross0, 0.0, scen.payload, softness_of(terrain, 0.0))
        self.agent.set_normals(normals)

        result = RunResult(scenario=scen.name, policy=self.policy, terrain=terrain.name)
        t = 0.0
        a_long_prev = 0.0
        a_lat_prev = 0.0
        f_drive_prev = 0.0
        torque_util_prev = 0.0
        crr_prev = base_crr(terrain, 0.0)
        stop_time: float | None = None
        stall_events = 0
        last_intervention = -10.0

        while t < scen.duration:
            grade = terrain.grade(state.s)
            cross = terrain.cross_slope(state.s)
            curvature = terrain.curve(state.s)

            frame = self.sensors.read(
                t,
                state.s,
                state.y,
                terrain,
                state.pitch,
                state.roll,
                state.yaw_rate,
                a_long_prev,
                a_lat_prev,
                state.omega,
                max(state.v, 0.0),
                state.load,
            )

            # 坡道驻车场景：到达指定位置后驻车，再按设定时间重新起步
            stop_request = False
            if scen.stop_at is not None and state.s >= scen.stop_at:
                if stop_time is None:
                    stop_time = t
                if scen.restart_after is None or t < stop_time + scen.restart_after:
                    stop_request = True

            info = {
                "grade": grade,
                "cross": cross,
                "curvature": curvature,
                "a_long": a_long_prev,
                "f_drive": f_drive_prev,
                "crr": crr_prev,
                "torque_util": torque_util_prev,
            }

            torque, brake, parked, diag = self.agent.step(frame, state, info, dt, stop_request)

            softness = softness_of(terrain, state.s)
            state, forces, veh_info = self.model.step(
                state,
                terrain,
                torque,
                brake,
                a_lat_prev,
                softness,
                dt,
                parked=parked,
            )
            self.agent.set_normals(forces.normal)

            a_long_prev = veh_info["a_long"]
            a_lat_prev = state.v ** 2 * curvature
            f_drive_prev = veh_info["f_drive"]
            torque_util_prev = float(np.max(np.abs(torque)) / vp.max_wheel_torque)
            crr_prev = veh_info["crr"]

            if diag.safety.code != "OK" and t - last_intervention > 0.5:
                stall_events += int(diag.safety.code in ("STALL_RECOVERY", "BLOCKED_AHEAD", "STABILITY_CRITICAL"))
                last_intervention = t

            result.rows.append({
                "t": round(t, 3),
                "s": round(state.s, 3),
                "v_true": round(state.v, 3),
                "v_est": round(diag.v_est, 3),
                "grade": round(grade, 2),
                "cross": round(cross, 2),
                "roll": round(state.roll, 2),
                "pitch": round(state.pitch, 2),
                "slip_fl": round(float(forces.slip[0]), 3),
                "slip_fr": round(float(forces.slip[1]), 3),
                "slip_rl": round(float(forces.slip[2]), 3),
                "slip_rr": round(float(forces.slip[3]), 3),
                "slip_mean": round(float(np.mean(forces.slip)), 3),
                "mu_true": round(veh_info["mu_eff"], 3),
                "mu_est": round(diag.mu_est, 3),
                "torque": round(float(np.mean(torque)), 1),
                "brake": round(brake, 2),
                "v_limit": round(diag.traversability.v_limit, 3),
                "risk": round(diag.traversability.risk, 3),
                "roll_margin": round(diag.stability.roll_margin, 3),
                "energy": round(state.energy, 1),
                "distance": round(state.distance, 3),
                "mode": diag.mode,
                "soil": diag.soil_name,
                "blocked": int(diag.traversability.blocked),
                "safety": diag.safety.code,
                "hold_state": diag.hold.state,
                "hold_slip": round(self.agent.hold.slip_distance, 3),
            })

            if state.tipped:
                break
            if state.s < -5.0:
                # 后溜越过场景下边界：判定为失控，停止推演，避免数值上无限下滑
                break
            if state.s >= scen.length:
                break
            t += dt

        result.summary = summarize(result.rows, scen, state, terrain)
        result.summary["stall_events"] = stall_events
        return result


# ---------------------------------------------------------------------------
def summarize(rows: list[dict], scen: ScenarioParams, state: VehicleState, terrain: TerrainProfile) -> dict:
    if not rows:
        return {}
    slip = np.array([r["slip_mean"] for r in rows])
    roll = np.array([abs(r["roll"]) for r in rows])
    v = np.array([r["v_true"] for r in rows])
    mu_est = np.array([r["mu_est"] for r in rows])
    mu_err = np.abs(mu_est - np.array([r["mu_true"] for r in rows]))
    final = rows[-1]
    reached = state.s >= scen.length - 0.3
    max_s = max(r["s"] for r in rows)
    return {
        "scenario": scen.name,
        "terrain": terrain.name,
        "payload": scen.payload,
        "reached": bool(reached),
        "final_s": round(float(state.s), 2),
        "time_s": round(float(rows[-1]["t"]), 2),
        "mean_slip": round(float(slip.mean()), 4),
        "p95_slip": round(float(np.percentile(slip, 95)), 4),
        "slip_above_035_ratio": round(float(np.mean(slip > 0.35)), 3),
        "max_roll": round(float(roll.max()), 2),
        "roll_violation_ratio": round(float(np.mean(roll > 24.0)), 3),
        "tipped": bool(state.tipped),
        "min_speed": round(float(v.min()), 3),
        "back_slip_distance": round(float(max(0.0, max_s - final["s"])), 3),
        "energy_kwh": round(float(state.energy) / 3.6e6, 4),
        "energy_per_meter_wh": round(float(state.energy) / max(state.distance, 0.1) / 3600.0, 3),
        "mu_est_mae": round(float(mu_err.mean()), 4),
        "hold_slip_distance": round(float(final["hold_slip"]), 3),
        "safety_interventions": int(sum(1 for a, b in zip(rows, rows[1:]) if b["safety"] != "OK" and a["safety"] != b["safety"])),
        "blocked_ratio": round(float(np.mean([r["blocked"] for r in rows])), 3),
    }
