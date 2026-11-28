"""Jetson 侧定频节点：20 Hz 规划输出 + 心跳，供 H7 看门狗使用。

设计约束（对应 docs/prototype/04_控制与软件.md）：
- 规划周期 50 ms，超时即标记 deadline miss，不阻塞下一拍；
- 每拍必发心跳（序号自增），断流 200 ms 由 H7 判 HOLD；
- 通过 Transport 抽象屏蔽 socketcan / 台架回放，便于无车上测试。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Protocol

from slopehaul.deploy.can_protocol import Command, NODES, make_feedback


class Transport(Protocol):
    def send(self, can_id: int, data: bytes) -> None: ...


@dataclass
class RecordTransport:
    """测试用：把发出的帧记录到内存。"""

    sent: list[tuple[int, bytes]] = field(default_factory=list)

    def send(self, can_id: int, data: bytes) -> None:
        self.sent.append((can_id, data))

    @property
    def count(self) -> int:
        return len(self.sent)

    def ids(self) -> list[int]:
        return [i for i, _ in self.sent]


@dataclass
class NodeConfig:
    rate_hz: float = 20.0
    budget_ms: float = 50.0
    max_torque_nm: float = 190.0
    track_m: float = 1.10

    @property
    def dt(self) -> float:
        return 1.0 / self.rate_hz


@dataclass
class NodeStats:
    cycles: int = 0
    deadline_miss: int = 0
    max_compute_ms: float = 0.0
    last_seq: int = 0

    def as_dict(self) -> dict:
        return {
            "cycles": self.cycles,
            "deadline_miss": self.deadline_miss,
            "max_compute_ms": round(self.max_compute_ms, 2),
            "last_seq": self.last_seq,
        }


Planner = Callable[[float], tuple[dict[int, float], float]]


def skid_steer_mix(v_m_s: float, w_rad_s: float, cfg: NodeConfig, k_torque: float = 60.0) -> dict[int, float]:
    """差速混控：左 1/3、右 2/4，横向扭矩按轮距反算线速度。"""
    half = cfg.track_m / 2.0
    v_left = v_m_s - w_rad_s * half
    v_right = v_m_s + w_rad_s * half
    return {
        1: k_torque * v_left,
        3: k_torque * v_left,
        2: k_torque * v_right,
        4: k_torque * v_right,
    }


class JetsonNode:
    def __init__(
        self,
        planner: Planner,
        transport: Transport,
        cfg: NodeConfig | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.cfg = cfg or NodeConfig()
        self.planner = planner
        self.transport = transport
        self.stats = NodeStats()
        self._t = 0.0
        self._clock = clock

    def now(self) -> float:
        return self._clock() if self._clock else self._t

    def heartbeat(self) -> int:
        self.stats.last_seq = (self.stats.last_seq + 1) & 0xFFFF
        body = bytes([0x01, self.stats.last_seq & 0xFF, (self.stats.last_seq >> 8) & 0xFF, 0, 0, 0])
        from slopehaul.deploy.can_protocol import crc16_ccitt
        import struct

        self.transport.send(0x101, body + struct.pack("<H", crc16_ccitt(body)))
        return self.stats.last_seq

    def step(self, requested: dict[int, float] | None = None) -> dict[int, float]:
        """一拍：规划（或直接给定）→ 限幅 → 下发 4 帧 + 心跳。"""
        t0 = self.now()
        if requested is None:
            requested, compute_ms = self.planner(t0)
        else:
            compute_ms = (self.now() - t0) * 1000.0 + 1.0

        self.stats.cycles += 1
        self.stats.max_compute_ms = max(self.stats.max_compute_ms, compute_ms)
        if compute_ms > self.cfg.budget_ms:
            self.stats.deadline_miss += 1

        clamped = {n: max(-self.cfg.max_torque_nm, min(self.cfg.max_torque_nm, requested.get(n, 0.0))) for n in NODES}
        for node, torque in clamped.items():
            can_id, data = Command(node=node, torque_nm=torque, counter=self.stats.cycles & 0xFFFF).encode()
            self.transport.send(can_id, data)
        self.heartbeat()

        if self._clock is None:
            self._t += self.cfg.dt
        return clamped

    def run(self, cycles: int) -> NodeStats:
        for _ in range(cycles):
            self.step()
        return self.stats

    def report_feedback(self, node: int, speed_mm_s: int, current_a: float, temp_c: float, fault: int = 0) -> None:
        """台架自检用：构造并发送一帧驱动器反馈，便于上位机联调。"""
        can_id, data = make_feedback(node, speed_mm_s, current_a, temp_c, fault).encode()
        self.transport.send(can_id, data)
