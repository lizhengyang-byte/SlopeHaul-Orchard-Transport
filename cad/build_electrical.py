"""电气图纸 2 张 + 线束表（DXF/PDF/PNG）。

SLH-P1-E01 主回路（48V 母线→四轮毂电机/DC-DC）
SLH-P1-E02 控制与通信拓扑（Jetson Orin Nano ↔ STM32H7 ↔ 传感器/执行器）
运行：python -m cad.build_electrical
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

from ezdxf.enums import TextEntityAlignment

from cad.dwg_common import ascii_only, export, notes, new_sheet, rect, rrect, txt

OUT = Path(__file__).resolve().parent / "out"


def dot(msp, x, y, r=0.7):
    msp.add_circle((x, y), r, dxfattribs={"layer": "WIRE"})


def box(msp, x, y, w, h, label="", sub="", layer="OUTLINE", h1=2.8, h2=2.2):
    rect(msp, x, y, w, h, layer)
    if label:
        txt(msp, label, x + w / 2, y + h / 2 + (3.0 if sub else -1.2), h1, "TEXT", TextEntityAlignment.CENTER)
        if sub:
            txt(msp, sub, x + w / 2, y + h / 2 - 3.6, h2, "TEXT", TextEntityAlignment.CENTER)


def wire(msp, pts, layer="WIRE"):
    msp.add_lwpolyline(pts, dxfattribs={"layer": layer})


def fuse(msp, x, y, label="", h=6.0):
    """水平母线上的熔断器：方框 + 贯穿线。"""
    rect(msp, x - h / 2, y - 1.0, h, 2.0, layer="POWER")
    msp.add_line((x, y - 1.0), (x, y + 1.0), dxfattribs={"layer": "POWER"})
    if label:
        txt(msp, label, x - 3, y + 2.6, 2.2, "TEXT")


def contactor(msp, x, y, label=""):
    """接触器主触点：两竖条 + 斜拉杆。"""
    msp.add_line((x - 1.5, y - 2), (x - 1.5, y + 2), dxfattribs={"layer": "POWER"})
    msp.add_line((x + 1.5, y - 2), (x + 1.5, y + 2), dxfattribs={"layer": "POWER"})
    msp.add_line((x - 1.5, y - 2.6), (x + 1.5, y + 2.6), dxfattribs={"layer": "HIDDEN"})
    if label:
        txt(msp, label, x - 4, y + 3.2, 2.2, "TEXT")


def gnd(msp, x, y):
    for i, w in enumerate((6, 4, 2)):
        msp.add_line((x - w / 2, y - i * 1.6), (x + w / 2, y - i * 1.6), dxfattribs={"layer": "GND"})


# ---------------------------------------------------------------- E01 主回路
def draw_power(msp):
    bus_y, ret_y = 218, 150
    box(msp, 30, 168, 50, 50, "BATTERY", "48V 50Ah 2.4kWh")
    txt(msp, "+", 74, 212, 4.0, "POWER")
    txt(msp, "-", 74, 152, 4.0, "GND")
    msp.add_line((82, bus_y), (404, bus_y), dxfattribs={"layer": "POWER"})
    msp.add_line((82, ret_y), (404, ret_y), dxfattribs={"layer": "GND"})
    txt(msp, "48V BUS  J1-A", 176, bus_y + 4, 2.5, "POWER")
    txt(msp, "0V RETURN  J1-B", 300, ret_y - 5, 2.5, "GND")

    fuse(msp, 96, bus_y, "F1 125A")
    contactor(msp, 112, bus_y, "K1")
    # 预充支路
    msp.add_line((103, bus_y), (103, 208), dxfattribs={"layer": "POWER"})
    fuse(msp, 103, 208, "R1 100R")
    msp.add_line((103, 208), (112, 208), dxfattribs={"layer": "POWER"})

    loads = [
        (140, "M1", "48V 400W", "FR-L"),
        (186, "M2", "48V 400W", "FR-R"),
        (232, "M3", "48V 400W", "RR-L"),
        (278, "M4", "48V 400W", "RR-R"),
        (324, "DC24", "24V 10A", "JETSON"),
        (370, "DC12", "12V 5A", "LAMP"),
    ]
    for x, tag, spec, sub in loads:
        fuse(msp, x + 20, 212, "30A" if tag.startswith("M") else "10A")
        msp.add_line((x + 20, bus_y), (x + 20, 204), dxfattribs={"layer": "POWER"})
        rect(msp, x, 170, 40, 34)
        txt(msp, tag, x + 20, 194, 3.0, "TEXT", TextEntityAlignment.CENTER)
        txt(msp, spec, x + 20, 185, 2.2, "TEXT", TextEntityAlignment.CENTER)
        txt(msp, sub, x + 20, 177, 2.0, "TEXT", TextEntityAlignment.CENTER)
        msp.add_line((x + 20, 170), (x + 20, ret_y), dxfattribs={"layer": "POWER"})

    # 主接触器线圈 + 急停链路
    msp.add_line((112, bus_y), (112, 238), dxfattribs={"layer": "POWER"})
    msp.add_line((112, 238), (26, 238), dxfattribs={"layer": "POWER"})
    msp.add_line((26, 238), (26, 118), dxfattribs={"layer": "POWER"})
    box(msp, 26, 96, 66, 18, "E-STOP x2 NC", "J5 1-2 / 3-4", layer="HOLES")
    msp.add_line((26, 96), (26, 92), dxfattribs={"layer": "POWER"})
    box(msp, 26, 62, 80, 26, "K1 COIL", "via Q1 (H7)")
    msp.add_line((38, 62), (38, 58), dxfattribs={"layer": "GND"})
    gnd(msp, 38, 58)
    txt(msp, "HARDWARE CHAIN: E-STOP(NC x2) -> SAFETY RELAY -> K1 ; Q1 IN SERIES", 26, 50, 2.5)

    gnd(msp, 404, ret_y - 2)
    txt(msp, "POWER CIRCUIT  NTS", 130, 66, 3.5)
    notes(msp, [
        "1. 48V BUS: 4 mm2, PEAK 60 A, DROP <0.5 V @10 m",
        "2. EACH HUB MOTOR: 30 A FAST FUSE; DC-DC BRANCH: 10 A",
        "3. K1 NORMALLY OPEN, COIL VIA H7 Q1 + WATCHDOG RELAY",
        "4. E-STOP DUAL CHANNEL (NC x2) CUTS K1 IN HARDWARE",
        "5. PRECHARGE R1 100R/50W, CLOSE K1 AFTER 300 ms",
    ], y=280)


# ---------------------------------------------------------------- E02 控制拓扑
def draw_control(msp):
    box(msp, 84, 186, 80, 50, "STM32H723", "400MHz / FDCAN x2")
    box(msp, 190, 186, 80, 50, "JETSON ORIN NANO", "8GB / Ubuntu22 / ROS2")

    # CAN 总线
    msp.add_line((124, 186), (124, 166), dxfattribs={"layer": "SIGNAL"})
    msp.add_line((230, 186), (230, 166), dxfattribs={"layer": "SIGNAL"})
    msp.add_line((100, 166), (330, 166), dxfattribs={"layer": "SIGNAL"})
    txt(msp, "CAN 2.0B 500 kbps  120R x2", 132, 170, 2.5, "SIGNAL")

    for i, (tag, name) in enumerate([("M1", "FR-L"), ("M2", "FR-R"), ("M3", "RR-L"), ("M4", "RR-R"), ("BMS", "48V PACK")]):
        bx = 84 + i * 52
        box(msp, bx, 116, 46, 24, tag, name, h1=2.6, h2=2.0)
        msp.add_line((bx + 23, 140), (bx + 23, 166), dxfattribs={"layer": "SIGNAL"})

    # Jetson 侧传感器
    for i, (tag, sub, j) in enumerate([("LiDAR", "MID-360 / ETH", "J3"), ("RTK", "UM982 / UART", "J4"), ("CAM-F", "USB3 1080p", "J6"), ("CAM-R", "USB3", "J7")]):
        cy = 250 - i * 34
        box(msp, 336, cy - 11, 64, 22, tag, sub, h1=2.6, h2=2.0)
        msp.add_lwpolyline([(336, cy), (302, cy), (302, 210), (270, 210)], dxfattribs={"layer": "SIGNAL"})
        txt(msp, j, 306, cy - 4, 2.2)

    # H7 侧执行器与输入
    for i, (tag, sub, j) in enumerate([("E-STOP", "DI x2 NC", "J5"), ("BUMPER", "DI x2", "J12"), ("IMU", "SPI 200Hz", "J11"), ("ENC x4", "ABZ", "J10"), ("K1 RELAY", "DO", "J8"), ("BUZZER", "DO", "J9")]):
        cy = 250 - i * 34
        box(msp, 14, cy - 11, 56, 22, tag, sub, h1=2.4, h2=2.0)
        msp.add_lwpolyline([(70, cy), (78, cy), (78, 210), (84, 210)], dxfattribs={"layer": "SIGNAL"})
        txt(msp, j, 14, cy - 15, 2.2)

    txt(msp, "SAFETY: H7 200Hz HEARTBEAT -> JETSON ; >200ms LOST -> HOLD", 84, 96, 2.5)
    txt(msp, "CONTROL: JETSON 20Hz PATH -> H7 200Hz LOOP -> DRIVER 500Hz", 84, 89, 2.5)
    txt(msp, "CONTROL TOPOLOGY  NTS", 14, 50, 3.5)

    notes(msp, [
        "1. POWER: J1 48V -> DC24 -> JETSON 19V / LIDAR 12V / RTK 5V",
        "2. CAN NODES: 4 DRIVERS + BMS, 500 kbps, DUAL 120R",
        "3. SENSORS ON JETSON ONLY; REAL-TIME LOOP CLOSED IN H7",
        "4. E-STOP DUAL CHANNEL -> H7 DI + HARD CUT K1 (SEE E01)",
        "5. CONNECTORS GX16 / JST-XH, CODED J1..J12 (HARNESS CSV)",
    ], x=196, y=108)


def build(title, code, info, draw_fn, fname):
    doc, msp = new_sheet(title, code, info, scale="NTS")
    draw_fn(msp)
    ascii_only(msp)
    return export(doc, OUT / fname)


HARNESS = [
    ("J1", "Battery 48V", "PDB", 1, "48V+", "4.0", 400, "主电源正，红"),
    ("J1", "Battery 48V", "PDB", 2, "0V", "4.0", 400, "主电源负，黑"),
    ("J2", "STM32H7", "CAN BUS", 1, "CAN_H", "0.5", 300, "双绞，120R 终端"),
    ("J2", "STM32H7", "CAN BUS", 2, "CAN_L", "0.5", 300, "双绞"),
    ("J3", "Jetson", "LiDAR", 1, "ETH_TX+", "0.3", 500, "屏蔽 Cat6"),
    ("J3", "Jetson", "LiDAR", 2, "ETH_TX-", "0.3", 500, "屏蔽 Cat6"),
    ("J4", "Jetson", "RTK", 1, "UART_TX", "0.3", 600, "3.3V 电平"),
    ("J4", "Jetson", "RTK", 2, "UART_RX", "0.3", 600, "3.3V 电平"),
    ("J5", "E-STOP x2", "STM32H7", 1, "ESTOP_CH1", "0.5", 800, "NC，断线即停"),
    ("J5", "E-STOP x2", "STM32H7", 2, "ESTOP_CH2", "0.5", 800, "NC，独立通道"),
    ("J6", "Jetson", "CAM-F", 1, "USB3_DP", "0.3", 700, "前视"),
    ("J7", "Jetson", "CAM-R", 1, "USB3_DP", "0.3", 700, "后视"),
    ("J8", "STM32H7", "SAFETY RELAY", 1, "K1_EN", "0.5", 300, "DO，低有效"),
    ("J10", "Wheel Encoder x4", "STM32H7", 1, "ENC_A", "0.3", 1200, "ABZ 差分"),
    ("J11", "IMU", "STM32H7", 1, "SPI_SCK", "0.3", 400, "200 Hz"),
    ("J12", "Bumper x2", "STM32H7", 1, "BUMP_L", "0.5", 600, "DI 上拉"),
]

if __name__ == "__main__":
    build("POWER CIRCUIT", "SLH-P1-E01", "HARNESS + PDB", draw_power, "SLH-P1-E01_POWER CIRCUIT")
    build("CONTROL TOPOLOGY", "SLH-P1-E02", "ECU + SENSORS", draw_control, "SLH-P1-E02_CONTROL TOPOLOGY")
    csv_path = OUT / "harness_slopehaul_p1.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["连接器", "起点", "终点", "针脚", "信号", "线径mm2", "长度mm", "备注"])
        w.writerows(HARNESS)
    print("saved", csv_path.name, len(HARNESS), "rows")
