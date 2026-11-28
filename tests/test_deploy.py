"""deploy 层单元测试：协议往返、CRC 校验、看门狗、急停锁存、定频节拍。"""

from __future__ import annotations

import struct

import pytest

from slopehaul.control.failsafe import Level
from slopehaul.deploy.can_protocol import (
    Command,
    ProtocolError,
    crc16_ccitt,
    decode_command,
    decode_feedback,
    make_feedback,
)
from slopehaul.deploy.jetson_node import JetsonNode, NodeConfig, RecordTransport, skid_steer_mix
from slopehaul.deploy.supervisor import Supervisor, SupervisorConfig


def test_crc_reference_value():
    # CRC-16/CCITT-FALSE 对 "123456789" 的标准校验值 0x29B1
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_command_roundtrip_all_nodes():
    for node in (1, 2, 3, 4):
        cmd = Command(node=node, torque_nm=-42.5, counter=1234)
        can_id, data = cmd.encode()
        assert len(data) == 8
        back = decode_command(can_id, data)
        assert back.node == node
        assert back.torque_nm == pytest.approx(-42.5)
        assert back.counter == 1234 and back.enable is True


def test_command_crc_detects_bitflip():
    can_id, data = Command(node=2, torque_nm=30.0).encode()
    bad = bytearray(data)
    bad[1] ^= 0x01
    with pytest.raises(ProtocolError):
        decode_command(can_id, bytes(bad))


def test_torque_clamped_to_motor_rating():
    assert Command(node=1, torque_nm=500.0).clamp().torque_nm == 190.0
    assert Command(node=1, torque_nm=-500.0).clamp().torque_nm == -190.0


def test_feedback_roundtrip():
    can_id, data = make_feedback(3, speed_mm_s=-1500, current_a=12.4, temp_c=48, fault=0).encode()
    fb = decode_feedback(can_id, data)
    assert fb.node == 3 and fb.speed_mm_s == -1500
    assert fb.current_a == pytest.approx(12.4)
    assert fb.temp_c == 48 and not fb.faulted


def test_supervisor_holds_on_missing_heartbeat():
    sup = Supervisor(SupervisorConfig(heartbeat_timeout_ms=200))
    sup.on_heartbeat(0.0, 1)
    cmd = sup.step(0.0, {1: 80.0, 2: 80.0})
    assert cmd[1].enable is True
    # 心跳断流 0.3 s → HOLD，扭矩归零
    sup.on_heartbeat(0.0, 1)
    stall = sup.step(0.3, {1: 80.0})
    assert stall[1].torque_nm == pytest.approx(0.0)
    assert sup.stats.last_level == "HOLD"


def test_supervisor_estop_latches_until_operator_ack():
    sup = Supervisor()
    sup.on_heartbeat(0.0, 1)
    sup.trip_estop()
    out = sup.step(0.01, {1: 100.0})
    assert out[1].torque_nm == 0.0 and out[1].enable is False
    assert sup.estop_latched is True
    assert sup.clear_estop(operator_ack=False) is False
    assert sup.estop_latched is True
    assert sup.clear_estop(operator_ack=True) is True
    sup.on_heartbeat(0.02, 2)
    assert sup.step(0.02, {1: 10.0})[1].enable is True


def test_supervisor_slew_limits_step_change():
    cfg = SupervisorConfig(slew_nm_per_s=2000.0, loop_hz=200)
    sup = Supervisor(cfg)
    for i in range(10):
        sup.on_heartbeat(i * cfg.dt, i)
    prev = 0.0
    for i in range(10, 30):
        sup.on_heartbeat(i * cfg.dt, i)
        cmd = sup.step(i * cfg.dt, {1: 190.0})[1].torque_nm
        assert cmd - prev <= 2000.0 * cfg.dt + 1e-9
        prev = cmd


def test_skid_steer_mix_signs():
    cfg = NodeConfig(track_m=1.10)
    left_turn = skid_steer_mix(0.5, 0.3, cfg)
    assert left_turn[1] < left_turn[2]   # 左转：左侧轮慢、右侧轮快
    straight = skid_steer_mix(0.5, 0.0, cfg)
    assert straight[1] == straight[3] and straight[2] == straight[4]


def test_jetson_node_emits_4_cmd_plus_heartbeat():
    cfg = NodeConfig(rate_hz=20)
    tr = RecordTransport()
    node = JetsonNode(lambda t: ({1: 5.0, 2: 5.0, 3: 5.0, 4: 5.0}, 3.0), tr, cfg)
    stats = node.run(10)
    assert stats.cycles == 10
    assert tr.count == 10 * 5          # 4 命令 + 1 心跳
    assert 0x101 in tr.ids()


def test_jetson_node_flags_deadline_miss():
    cfg = NodeConfig(rate_hz=20, budget_ms=50.0)
    tr = RecordTransport()
    node = JetsonNode(lambda t: ({1: 0.0}, 80.0), tr, cfg)
    stats = node.run(5)
    assert stats.deadline_miss == 5
    assert stats.max_compute_ms == 80.0


def test_heartbeat_counter_wraps():
    tr = RecordTransport()
    node = JetsonNode(lambda t: ({}, 1.0), tr, NodeConfig())
    node.run(2)
    hb = [d for i, d in tr.sent if i == 0x101]
    seq = [struct.unpack("<H", d[1:3])[0] for d in hb]
    assert seq == [1, 2]
