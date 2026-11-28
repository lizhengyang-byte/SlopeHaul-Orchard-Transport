"""CAN 应用层协议（SLH-CAN v1）：命令帧 / 反馈帧 + CRC-16/CCITT。

上车即用：H7 侧 200 Hz 下发命令（0x180+n），驱动器 500 Hz 回反馈（0x200+n）。
字段定义与 docs/prototype/03_电气与BOM.md、04_控制与软件.md 对齐。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

DLC = 8
CMD_BASE_ID = 0x180
FB_BASE_ID = 0x200
TORQUE_LSB_NM = 0.1      # 命令扭矩分辨率 0.1 N·m
SPEED_LSB_MM_S = 1.0     # 反馈转速分辨率 1 mm/s
MAX_TORQUE_NM = 190.0    # 轮毂电机额定 190 N·m
NODES = (1, 2, 3, 4)     # FR-L / FR-R / RR-L / RR-R


class ProtocolError(ValueError):
    """帧格式、长度或 CRC 不合法。"""


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE（poly 0x1021, init 0xFFFF），逐位实现，无查表。"""

    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


@dataclass(frozen=True)
class Command:
    """下发给单个轮毂驱动器的命令帧语义。"""

    node: int
    torque_nm: float
    enable: bool = True
    mode: int = 1          # 0=待机 1=扭矩 2=转速
    counter: int = 0       # 0..65535，丢帧检测

    def clamp(self) -> "Command":
        t = max(-MAX_TORQUE_NM, min(MAX_TORQUE_NM, self.torque_nm))
        return Command(self.node, t, self.enable, self.mode, self.counter)

    def encode(self) -> tuple[int, bytes]:
        if self.node not in NODES:
            raise ProtocolError(f"非法节点号 {self.node}")
        body = struct.pack(
            "<BhBH",
            self.mode & 0xFF,
            int(round(self.torque_nm / TORQUE_LSB_NM)),
            1 if self.enable else 0,
            self.counter & 0xFFFF,
        )
        return CMD_BASE_ID + self.node, body + struct.pack("<H", crc16_ccitt(body))


@dataclass(frozen=True)
class Feedback:
    node: int
    speed_mm_s: int
    current_a: float
    temp_c: float
    fault: int

    @property
    def faulted(self) -> bool:
        return self.fault != 0

    def decode(self) -> dict:
        return {
            "node": self.node,
            "speed_mm_s": self.speed_mm_s,
            "current_a": round(self.current_a, 1),
            "temp_c": round(self.temp_c, 1),
            "fault": self.fault,
        }


def decode_command(can_id: int, data: bytes) -> Command:
    if len(data) != DLC:
        raise ProtocolError(f"DLC={len(data)}，应为 {DLC}")
    node = can_id - CMD_BASE_ID
    if node not in NODES:
        raise ProtocolError(f"非法命令 ID 0x{can_id:03X}")
    body, crc = data[:6], struct.unpack("<H", data[6:])[0]
    if crc16_ccitt(body) != crc:
        raise ProtocolError("命令帧 CRC 校验失败")
    mode, raw_t, enable, counter = struct.unpack("<BhBH", body)
    return Command(node, raw_t * TORQUE_LSB_NM, bool(enable), mode, counter)


@dataclass(frozen=True)
class FeedbackFrame:
    node: int
    body: bytes

    def encode(self) -> tuple[int, bytes]:
        if self.node not in NODES:
            raise ProtocolError(f"非法节点号 {self.node}")
        return FB_BASE_ID + self.node, self.body + struct.pack("<H", crc16_ccitt(self.body))


def make_feedback(node: int, speed_mm_s: int, current_a: float, temp_c: float, fault: int = 0) -> FeedbackFrame:
    body = struct.pack("<BhBBb", node & 0xFF, int(speed_mm_s), int(current_a * 10) & 0xFF, int(temp_c) & 0xFF, fault & 0xFF)
    return FeedbackFrame(node, body)


def decode_feedback(can_id: int, data: bytes) -> Feedback:
    if len(data) != DLC:
        raise ProtocolError(f"DLC={len(data)}，应为 {DLC}")
    node = can_id - FB_BASE_ID
    if node not in NODES:
        raise ProtocolError(f"非法反馈 ID 0x{can_id:03X}")
    body, crc = data[:6], struct.unpack("<H", data[6:])[0]
    if crc16_ccitt(body) != crc:
        raise ProtocolError("反馈帧 CRC 校验失败")
    node_b, speed, cur_raw, temp, fault = struct.unpack("<BhBBb", body)
    return Feedback(node_b, speed, cur_raw / 10.0, float(temp), fault)
