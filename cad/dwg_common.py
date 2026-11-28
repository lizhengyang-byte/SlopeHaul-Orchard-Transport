"""图纸公共件：A3 图框、标题栏、标注与文字工具（供 build_drawings / build_electrical 共用）。

约定：纸张坐标（A3 420×297，单位 mm），零件几何按比例缩放后落纸；文字默认 3 mm。
"""

from __future__ import annotations

import ezdxf
from ezdxf.enums import TextEntityAlignment

A3_W, A3_H = 420, 297
FRAME = 8

LAYERS = {
    "OUTLINE": 7,
    "HIDDEN": 8,
    "CENTER": 1,
    "DIM": 3,
    "TEXT": 7,
    "TITLE": 5,
    "HOLES": 6,
    "WIRE": 4,     # 电气：导线
    "POWER": 2,    # 电气：电源正
    "GND": 8,      # 电气：地
    "SIGNAL": 6,   # 电气：信号
}


def new_sheet(title: str, code: str, info: str, scale: str = "1:10"):
    """建 A3 图框 + 4 行标题栏，返回 (doc, msp)。"""
    doc = ezdxf.new("R2010", setup=True)
    doc.header["$INSUNITS"] = 4
    doc.styles.add("CN", font="simsun.ttc")
    for name, color in LAYERS.items():
        doc.layers.add(name, color=color)
    msp = doc.modelspace()
    msp.add_lwpolyline([(0, 0), (A3_W, 0), (A3_W, A3_H), (0, A3_H)], close=True, dxfattribs={"layer": "TITLE"})
    msp.add_lwpolyline(
        [(FRAME, FRAME), (A3_W - FRAME, FRAME), (A3_W - FRAME, A3_H - FRAME), (FRAME, A3_H - FRAME)],
        close=True, dxfattribs={"layer": "TITLE"},
    )

    x0, y0, w, h = 230, 8, 182, 40
    msp.add_lwpolyline([(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)], close=True, dxfattribs={"layer": "TITLE"})
    for i in (1, 2, 3):
        msp.add_line((x0, y0 + i * 10), (x0 + w, y0 + i * 10), dxfattribs={"layer": "TITLE"})
    msp.add_line((x0 + 120, y0), (x0 + 120, y0 + h), dxfattribs={"layer": "TITLE"})
    rows = [
        (f"TITLE {title}", f"DWG NO {code}"),
        (f"MATERIAL {info}", f"SCALE {scale}"),
        ("DESIGNED L.SHANG RU  2026-11-28", "REV A1   UNIT mm"),
        ("SlopeHaul-P1  Chengdu Jincheng College", "SHEET 1 OF 1"),
    ]
    for i, (a, b) in enumerate(rows):
        msp.add_text(a, height=3.0, dxfattribs={"layer": "TEXT", "style": "CN"}).set_placement((x0 + 3, y0 + 31 - i * 10))
        msp.add_text(b, height=3.0, dxfattribs={"layer": "TEXT", "style": "CN"}).set_placement((x0 + 123, y0 + 31 - i * 10))
    return doc, msp


def rect(msp, ox, oy, w, h, layer="OUTLINE"):
    msp.add_lwpolyline([(ox, oy), (ox + w, oy), (ox + w, oy + h), (ox, oy + h)], close=True, dxfattribs={"layer": layer})


def rrect(msp, cx, cy, w, h, layer="OUTLINE"):
    """以中心定位的矩形（电气符号用）。"""
    rect(msp, cx - w / 2, cy - h / 2, w, h, layer)


def txt(msp, s, x, y, h=3.0, layer="TEXT", align=None):
    t = msp.add_text(s, height=h, dxfattribs={"layer": layer, "style": "CN"})
    t.set_placement((x, y), align=align or TextEntityAlignment.LEFT)
    return t


def dim_h(msp, x1, x2, y, text):
    msp.add_line((x1, y - 3), (x1, y + 3), dxfattribs={"layer": "DIM"})
    msp.add_line((x2, y - 3), (x2, y + 3), dxfattribs={"layer": "DIM"})
    msp.add_line((x1, y), (x2, y), dxfattribs={"layer": "DIM"})
    txt(msp, str(text), (x1 + x2) / 2, y + 1.6, 3.5, "DIM", TextEntityAlignment.CENTER)


def dim_v(msp, y1, y2, x, text):
    msp.add_line((x - 3, y1), (x + 3, y1), dxfattribs={"layer": "DIM"})
    msp.add_line((x - 3, y2), (x + 3, y2), dxfattribs={"layer": "DIM"})
    msp.add_line((x, y1), (x, y2), dxfattribs={"layer": "DIM"})
    txt(msp, str(text), x - 2, (y1 + y2) / 2 - 1, 3.5, "DIM", TextEntityAlignment.RIGHT)


def notes(msp, lines, x=226, y=272, title="NOTES:"):
    txt(msp, title, x, y, 3.5)
    for i, line in enumerate(lines):
        txt(msp, line, x, y - 7 - i * 6.5, 3.0)


def ascii_only(msp):
    """预览字体不含 Ø/±，统一替换为 DIA / +/-，避免 PDF 出方块。"""
    for e in msp.query("TEXT"):
        e.dxf.text = e.dxf.text.replace("Ø", "DIA").replace("±", "+/-")


def export(doc, path_stem, preview=True):
    """存 DXF，可选出 PDF/PNG 白底预览。"""
    from pathlib import Path

    p = Path(f"{Path(str(path_stem)).parent / Path(str(path_stem)).name}.dxf")
    doc.saveas(p)
    if not preview:
        return p
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from ezdxf.addons.drawing import Frontend, RenderContext
        from ezdxf.addons.drawing.config import BackgroundPolicy, ColorPolicy, Configuration
        from ezdxf.addons.drawing.matplotlib import MatplotlibBackend

        ctx = RenderContext(doc)
        cfg = Configuration(background_policy=BackgroundPolicy.WHITE, color_policy=ColorPolicy.MONOCHROME_LIGHT_BG)
        fig = plt.figure(figsize=(11.69, 8.27))
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_facecolor("white")
        Frontend(ctx, MatplotlibBackend(ax), config=cfg).draw_layout(doc.modelspace(), finalize=True)
        fig.savefig(p.with_suffix(".pdf"), facecolor="white")
        fig.savefig(p.with_suffix(".png"), dpi=180, facecolor="white")
        plt.close(fig)
    except Exception as exc:  # 预览失败不影响 DXF 交付
        print("preview skipped:", exc)
    return p
