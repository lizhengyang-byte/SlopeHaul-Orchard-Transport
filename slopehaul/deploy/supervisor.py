"""H7 侧 200 Hz 监督器：心跳看门狗 + 扭矩限幅/斜率 + 急停锁存。

职责边界：
- 只信任"最近心跳时间戳 + 序号"，不解析上层内容；
- 任何异常先把扭矩归零，再按 OK / DERATE / HOLD / STOP 分级；
- 急停属硬件链路（E-01 双通道切 K1），本模块负责软件侧一致与锁存。
"""

from __future__ import annotations

from dataclasses import dataclass

from slopehaul.control.failsafe import FailsafeManager, Level
from slopehaul.deploy.can_protocol import Command, MAX_TORQUE_NM, NODES


@dataclass
class SupervisorConfig:
    loop_hz: int = 200
    heartbeat_timeout_ms: int = 200
    slew_nm_per_s: float = 2000.0     # 扭矩斜率上限，抑制冲击
    max_torque_nm: float = MAX_TORQUE_NM

    @property
    def dt(self) -> float:
        return 1.0 / self.loop_hz


@dataclass
class SupervisorStats:
    ticks: int = 0
    holds: int = 0
    stops: int = 0
    derates: int = 0
    max_gap_ms: float = 0.0
    last_level: str = "OK"

    def as_dict(self) -> dict:
        return {
            "ticks": self.ticks,
            "holds": self.holds,
            "stops": self.stops,
            "derates": self.derates,
            "max_gap_ms": round(self.max_gap_ms, 1),
            "last_level": self.last_level,
        }


class Supervisor:
    """把"上层意图"变成"可下发的安全命令"。"""

    def __init__(self, cfg: SupervisorConfig | None = None, failsafe: FailsafeManager | None = None) -> None:
        self.cfg = cfg or SupervisorConfig()
        self.failsafe = failsafe or FailsafeManager()
        # 统一口径：看门狗阈值以 SupervisorConfig 为准（H7 侧 200 ms）
        self.failsafe.cfg.heartbeat_timeout_s = self.cfg.heartbeat_timeout_ms / 1000.0
        self.stats = SupervisorStats()
        self._last_cmd: dict[int, float] = {n: 0.0 for n in NODES}
        self._hb_t: float | None = None
        self._hb_seq: int | None = None
        self._last_tick: float | None = None
        self._estop = False

    # ---- 输入 ----
    def on_heartbeat(self, now_s: float, seq: int) -> bool:
        """记录心跳；序号回退视为异常（返回 False 且不刷新）。"""
        ok = self._hb_seq is None or seq >= self._hb_seq
        if ok:
            self._hb_t, self._hb_seq = now_s, seq
        return ok

    def heartbeat_age_ms(self, now_s: float) -> float:
        if self._hb_t is None:
            return float("inf")
        return (now_s - self._hb_t) * 1000.0

    # ---- 决策 ----
    def step(
        self,
        now_s: float,
        requested: dict[int, float],
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        motor_temp_c: float = 25.0,
        estop: bool | None = None,
    ) -> dict[int, Command]:
        """一个 5 ms 控制拍：返回 4 个节点的安全命令。"""
        self.stats.ticks += 1
        if self._last_tick is not None:
            self.stats.max_gap_ms = max(self.stats.max_gap_ms, (now_s - self._last_tick) * 1000.0)
        self._last_tick = now_s

        estop_in = self._estop if estop is None else (estop or self._estop)
        action = self.failsafe.step(
            now_s,
            self.heartbeat_age_ms(now_s) / 1000.0,
            s=0.0,
            y=0.0,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            motor_temp_c=motor_temp_c,
            estop=estop_in,
        )
        level = action.level
        self.stats.last_level = level.label
        if level is Level.HOLD:
            self.stats.holds += 1
        elif level is Level.STOP:
            self.stats.stops += 1
        elif level is Level.DERATE:
            self.stats.derates += 1

        out: dict[int, Command] = {}
        max_up = self.cfg.slew_nm_per_s * self.cfg.dt
        for node in NODES:
            if level in (Level.HOLD, Level.STOP):
                cur = 0.0  # 停机/保持不走斜率：看门狗与急停必须立即归零
            else:
                want = requested.get(node, 0.0) * action.speed_scale
                want = max(-self.cfg.max_torque_nm, min(self.cfg.max_torque_nm, want))
                prev = self._last_cmd[node]
                cur = prev + max(-max_up, min(max_up, want - prev))
            self._last_cmd[node] = cur
            out[node] = Command(node=node, torque_nm=cur, enable=level is not Level.STOP)
        return out

    # ---- 急停 ----
    def trip_estop(self) -> None:
        self._estop = True
        self._last_cmd = {n: 0.0 for n in NODES}

    def clear_estop(self, operator_ack: bool = True) -> bool:
        """只有操作员显式确认才允许复位；复位后仍要等心跳恢复。"""
        if not operator_ack:
            return False
        self._estop = False
        return self.failsafe.clear_estop(operator_ack=True)

    @property
    def estop_latched(self) -> bool:
        return self._estop
