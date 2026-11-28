"""deploy 层：CAN 协议、H7 监督器与 Jetson 定频节点（台架/实车共用）。"""

from slopehaul.deploy.can_protocol import (
    CMD_BASE_ID,
    FB_BASE_ID,
    Command,
    Feedback,
    ProtocolError,
    crc16_ccitt,
    decode_command,
    decode_feedback,
    make_feedback,
)
from slopehaul.deploy.jetson_node import JetsonNode, NodeConfig, NodeStats, RecordTransport, skid_steer_mix
from slopehaul.deploy.supervisor import Supervisor, SupervisorConfig, SupervisorStats

__all__ = [
    "CMD_BASE_ID",
    "FB_BASE_ID",
    "Command",
    "Feedback",
    "ProtocolError",
    "crc16_ccitt",
    "decode_command",
    "decode_feedback",
    "make_feedback",
    "JetsonNode",
    "NodeConfig",
    "NodeStats",
    "RecordTransport",
    "skid_steer_mix",
    "Supervisor",
    "SupervisorConfig",
    "SupervisorStats",
]
