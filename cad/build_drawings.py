"""生成机械图纸 4 张（DXF + PDF/PNG 预览）。

SLH-P1-01 车架总成 / 02 摆动桥 / 03 货箱 / 04 总装布局
运行：python -m cad.build_drawings
"""

from __future__ import annotations

import math
from pathlib import Path

from cad.dwg_common import ascii_only, dim_h, dim_v, export, notes, new_sheet, rect, txt

OUT = Path(__file__).resolve().parent / "out"

S = 0.1   # 1:10
S5 = 0.2  # 1:5
S2 = 0.5  # 2:1
L, W, WB, TRACK, WHEEL_R = 1620, 1100, 1250, 1100, 240
CLEAR, BODY_H = 260, 660
BED_W, BED_D, BED_H = 1050, 850, 280


# ---------------------------------------------------------------- 01 车架总成
def draw_frame(msp):
    ox, oy = 40, 150
    rect(msp, ox, oy, L * S, W * S)
    for yy in (oy + 9, oy + W * S - 9):  # 纵梁示意带
        msp.add_line((ox, yy), (ox + L * S, yy), dxfattribs={"layer": "OUTLINE"})
    for xx in (ox + 14, ox + 46, ox + 116, ox + 148):  # 横梁
        msp.add_line((xx, oy), (xx, oy + W * S), dxfattribs={"layer": "OUTLINE"})
    rect(msp, ox + 58, oy + 46, 44, 22, layer="HIDDEN")
    txt(msp, "BATTERY", ox + 61, oy + 55, 2.5)

    hp = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            px = ox + 60 + sx * (WB / 2 * 0.72) * S
            py = oy + 55 + sy * 380 * S
            rect(msp, px - 3.6, py - 3.0, 7.2, 6.0)
            msp.add_circle((px, py), 1.5, dxfattribs={"layer": "HOLES"})
            hp.append((px, py))
    dim_h(msp, ox, ox + L * S, oy + W * S + 6, 1620)
    dim_v(msp, oy, oy + W * S, ox + L * S + 8, 1100)
    txt(msp, "PLAN 1:10", ox + 6, oy - 10, 3.5)

    bx, by = 250, 118  # SECTION B-B
    rect(msp, bx, by, 80, 80)
    rect(msp, bx + 4, by + 4, 72, 72, layer="OUTLINE")
    dim_h(msp, bx, bx + 80, by + 84, "40 (2:1)")
    txt(msp, "SECTION B-B  RAIL 40x40x2", bx - 2, by - 10, 3.5)

    txt(msp, "HOLE TABLE (FROM LEFT-REAR CORNER, mm)", 16, 96, 3.5)
    for i, (px, py) in enumerate(sorted(hp, key=lambda p: (p[1], p[0]))):
        hx, hy = round((px - ox) / S), round((py - oy) / S)
        txt(msp, f"H{i + 1}   X={hx:4d}   Y={hy:4d}   Ø30 H7", 20, 89 - i * 6.5)


# ---------------------------------------------------------------- 02 摆动桥
def draw_swing(msp):
    px, py = 62, 215
    ax_, ay_ = px + 480 * S, py
    msp.add_line((px - 8, py), (ax_ + 8, py), dxfattribs={"layer": "CENTER"})
    for ang in (-12, 12):
        msp.add_line(
            (px, py),
            (px + 480 * S * math.cos(math.radians(ang)), py + 480 * S * math.sin(math.radians(ang))),
            dxfattribs={"layer": "HIDDEN"},
        )
    rect(msp, px, py - 2.5, 48, 5)
    msp.add_circle((px, py), 2.4, dxfattribs={"layer": "HOLES"})
    msp.add_circle((ax_, ay_), WHEEL_R * S, dxfattribs={"layer": "OUTLINE"})
    msp.add_circle((ax_, ay_), 1.25, dxfattribs={"layer": "HOLES"})
    for dy in (-9, 9):
        rect(msp, px + 8, py + dy - 1.5, 8, 3)
    # 弹簧/减震
    sx_, sy_ = px, py + 24
    msp.add_line((px, py + 2.4), (sx_, sy_), dxfattribs={"layer": "OUTLINE"})
    zz = [(sx_, sy_)]
    for k in range(6):
        zz.append((sx_ + (-4 if k % 2 == 0 else 4), sy_ + 4 + k * 3.5))
    msp.add_lwpolyline(zz, dxfattribs={"layer": "OUTLINE"})
    msp.add_line(zz[-1], (sx_, sy_ + 24), dxfattribs={"layer": "OUTLINE"})
    msp.add_line((sx_ - 6, sy_ + 24), (sx_ + 6, sy_ + 24), dxfattribs={"layer": "OUTLINE"})
    txt(msp, "SPRING 120 N/mm", sx_ + 8, sy_ + 18, 2.5)
    # 地面
    gy = py - WHEEL_R * S
    msp.add_line((30, gy), (195, gy), dxfattribs={"layer": "CENTER"})
    txt(msp, "GROUND", 176, gy + 2, 2.5)
    txt(msp, "ARM 480 C/C", px - 16, py - 13, 3.0)
    txt(msp, "SWING +/-12 DEG", px + 16, py + 34, 3.0, "DIM")
    txt(msp, "SIDE ELEVATION 1:10", px - 16, gy - 8, 3.5)

    qx, qy = 40, 120  # 俯视
    rect(msp, qx, qy, 620 * S, 150 * S)
    msp.add_circle((qx + 40 * S, qy + 75 * S), 1.5, dxfattribs={"layer": "HOLES"})
    msp.add_circle((qx + 580 * S, qy + 75 * S), 1.25, dxfattribs={"layer": "HOLES"})
    dim_h(msp, qx, qx + 620 * S, qy + 150 * S + 6, 620)
    dim_v(msp, qy, qy + 150 * S, qx + 620 * S + 8, 150)
    txt(msp, "PLAN 1:10", qx, qy - 8, 3.5)

    cx, cy = 300, 210  # DETAIL A 2:1
    msp.add_circle((cx, cy), 30 * S2, dxfattribs={"layer": "OUTLINE"})
    msp.add_circle((cx, cy), 18 * S2, dxfattribs={"layer": "HOLES"})
    msp.add_circle((cx, cy), 15 * S2, dxfattribs={"layer": "HOLES"})
    msp.add_line((cx - 18, cy), (cx + 18, cy), dxfattribs={"layer": "CENTER"})
    msp.add_line((cx, cy - 18), (cx, cy + 18), dxfattribs={"layer": "CENTER"})
    msp.add_line((cx + 10, cy + 11), (cx + 20, cy + 17), dxfattribs={"layer": "OUTLINE"})
    txt(msp, "DETAIL A 2:1", cx - 16, cy - 20, 3.5)
    txt(msp, "BOSS Ø60", cx + 22, cy + 12, 3.0)
    txt(msp, "HOLE Ø30 H7", cx + 22, cy + 5, 3.0)
    txt(msp, "SF BUSH Ø36/Ø30", cx + 22, cy - 2, 3.0)


# ---------------------------------------------------------------- 03 货箱
def draw_box(msp):
    ox, oy = 40, 172
    rect(msp, ox, oy, BED_W * S, BED_D * S)
    rect(msp, ox + 3, oy + 3, (BED_W - 60) * S, (BED_D - 60) * S, layer="HIDDEN")
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = ox + BED_W * S / 2 + sx * (BED_W / 2 - 60) * S
            cy = oy + BED_D * S / 2 + sy * (BED_D / 2 - 60) * S
            msp.add_circle((cx, cy), 1.8, dxfattribs={"layer": "HOLES"})
    msp.add_line((ox + BED_W * S / 2, oy), (ox + BED_W * S / 2, oy + BED_D * S), dxfattribs={"layer": "HIDDEN"})
    dim_h(msp, ox, ox + BED_W * S, oy + BED_D * S + 6, 1050)
    dim_v(msp, oy, oy + BED_D * S, ox + BED_W * S + 8, 850)
    txt(msp, "PLAN 1:10", ox - 2, oy - 8, 3.5)

    sx0, sy0 = 40, 62  # SECTION A-A 1:5
    rect(msp, sx0, sy0, BED_D * S5, BED_H * S5)
    msp.add_line((sx0 + 1, sy0 + 1), (sx0 + BED_D * S5 - 1, sy0 + 1), dxfattribs={"layer": "HIDDEN"})
    for fx in (sx0 + 8, sx0 + BED_D * S5 - 8):
        rect(msp, fx - 1.5, sy0 + BED_H * S5 - 3.5, 3, 3.5, layer="HOLES")
    txt(msp, "DECK 2.0", sx0 + 8, sy0 + 5, 2.5)
    txt(msp, "SIDE 1.5", sx0 + 3, sy0 + BED_H * S5 - 8, 2.5)
    dim_h(msp, sx0, sx0 + BED_D * S5, sy0 - 10, 850)
    dim_v(msp, sy0, sy0 + BED_H * S5, sx0 - 6, 280)
    txt(msp, "SECTION A-A 1:5   LATCH M8 x4", sx0, sy0 - 14, 3.5)


# ---------------------------------------------------------------- 04 总装布局
def draw_assembly(msp):
    ox, oy = 40, 160
    rect(msp, ox, oy, L * S, W * S)
    for xx in (ox + 14, ox + 148):
        msp.add_line((xx, oy), (xx, oy + W * S), dxfattribs={"layer": "HIDDEN"})
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = ox + L * S / 2 + sx * WB / 2 * S, oy + W * S / 2 + sy * TRACK / 2 * S
            msp.add_circle((cx, cy), WHEEL_R * S, dxfattribs={"layer": "OUTLINE"})
            msp.add_circle((cx, cy), 2.4, dxfattribs={"layer": "HIDDEN"})
    rect(msp, ox + 20, oy + 20, 44, 70, layer="HIDDEN")
    txt(msp, "BATTERY", ox + 22, oy + 52, 2.5)
    dim_h(msp, ox, ox + L * S, oy + W * S + 6, 1620)
    dim_v(msp, oy, oy + W * S, ox + L * S + 10, 1100)
    txt(msp, "PLAN 1:10  (FRONT UP)", ox + L * S + 12, oy + 4, 3.5)

    gy = 34
    rect(msp, ox, gy + CLEAR * S, L * S, BODY_H * S)
    msp.add_line((ox + 12, gy + (CLEAR + 280) * S), (ox + L * S - 12, gy + (CLEAR + 280) * S), dxfattribs={"layer": "HIDDEN"})
    txt(msp, "CARGO BOX 280 DEEP / 200 kg", ox + 14, gy + (CLEAR + 290) * S, 2.5)
    for sx in (-1, 1):
        msp.add_circle((ox + L * S / 2 + sx * WB / 2 * S, gy + WHEEL_R * S), WHEEL_R * S, dxfattribs={"layer": "OUTLINE"})
    msp.add_line((ox - 6, gy), (ox + L * S + 6, gy), dxfattribs={"layer": "CENTER"})
    my = gy + (CLEAR + BODY_H) * S
    msp.add_line((ox + L * S - 20, my), (ox + L * S - 20, my + 30), dxfattribs={"layer": "OUTLINE"})
    msp.add_circle((ox + L * S - 20, my + 32), 1.8, dxfattribs={"layer": "HOLES"})
    txt(msp, "LiDAR+RTK", ox + L * S - 34, my + 34, 2.5)
    msp.add_circle((ox + L * S / 2 + 3, gy + 60 * S), 0.8, dxfattribs={"layer": "CENTER"})
    txt(msp, "CoM 600 (FULL LOAD)", ox + L * S / 2 + 6, gy + 58 * S, 2.5)
    dim_v(msp, gy, gy + CLEAR * S, ox - 10, 260)
    txt(msp, "SIDE 1:10  (FRONT RIGHT)", ox + L * S + 12, gy + 18, 3.5)


def build(title, code, material, draw, note_lines):
    doc, msp = new_sheet(title, code, material)
    draw(msp)
    notes(msp, note_lines, y=276)
    ascii_only(msp)
    return export(doc, OUT / f"{code}_{title}")


def main():
    made = [
        build("FRAME ASSY", "SLH-P1-01", "6061-T6 40x40 + Q345", draw_frame,
              ["1. MAIN RAILS 40x40x2 ALU, DIAGONAL TOL <=2 mm (LASER + TIG)",
               "2. 4 SWING PIVOTS DIA30 H7, SEE HOLE TABLE",
               "3. STRESS RELIEF AFTER WELD, ANODIZE 15 um",
               "4. FRAME MASS 68 kg"]),
        build("SWING BOGIE", "SLH-P1-02", "Q345B WELDED", draw_swing,
              ["1. PIVOT DIA30 H7 + SF BUSH DIA36/30, AXLE DIA25 h6",
               "2. SWING +/-12 DEG, LIMIT BLOCK 45 STEEL HRC45",
               "3. SPRING 120 N/mm, PRELOAD 400 N, STROKE 60",
               "4. 4 SETS PER MACHINE (2 FRONT / 2 REAR)"]),
        build("CARGO BOX", "SLH-P1-03", "5052 ALU 2.0/1.5", draw_box,
              ["1. DECK 2.0 CHECKERED, SIDES 1.5 REMOVABLE, FULL WELD + GRIND",
               "2. 4x M8 LATCH + EPDM SEAL, 4x LOAD PAD DIA36",
               "3. CAPACITY 1050x850x280 = 0.25 m3 / 200 kg",
               "4. SANDBLAST + CLEAR ANODIZE"]),
        build("GEN ARRANGEMENT", "SLH-P1-04", "ASSEMBLY", draw_assembly,
              ["1. TRACK 1100, WHEELBASE 1250, WHEEL DIA 480, CLEARANCE 260",
               "2. KERB 320 kg + PAYLOAD 200 kg, CoM 600 (FULL)",
               "3. SENSOR MAST REMOVABLE, FOLD FOR TRANSPORT",
               "4. 4x4 HUB MOTOR 48V 400W, SKID STEER"]),
    ]
    for p in made:
        print("saved", p.name)


if __name__ == "__main__":
    main()
