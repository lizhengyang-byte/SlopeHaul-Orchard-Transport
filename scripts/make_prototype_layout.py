"""整机样机 P1 布局图（俯视 + 侧视 + 关键尺寸），输出 figures/prototype_layout.{png,svg}。

尺寸与 docs/prototype/02_机械设计 一致：1620×1100×920 mm，轴距 1250，轮距 1100，轮径 480，离地 260，货箱 1050×850×280。
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "figures"
OUT.mkdir(exist_ok=True)

plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
plt.rcParams["svg.hashsalt"] = "slopehaul"

INK = "#1f2933"
ACC = "#c2410c"
GREY = "#64748b"
W, L, H = 1.100, 1.620, 0.920
WHEEL_D = 0.480
WHEELBASE = 1.250
TRACK = 1.100
BED = (1.050, 0.850, 0.280)


def top_view(ax):
    ax.add_patch(FancyBboxPatch((-W / 2, -L / 2), W, L, boxstyle="round,pad=0.02", fc="#f8fafc", ec=INK, lw=1.4))
    ax.add_patch(Rectangle((-BED[1] / 2, -L / 2 + 0.16), BED[1], BED[0], fc="#fde68a", ec=ACC, lw=1.1, alpha=0.7))
    ax.text(0, 0.0, "货箱 1050×850", ha="center", va="center", fontsize=8, color=ACC)
    for sx in (-1, 1):
        for sy in (-1, 1):
            ax.add_patch(
                Rectangle((sx * TRACK / 2 - 0.07, sy * WHEELBASE / 2 - 0.24), 0.14, 0.48, fc=GREY, ec=INK, lw=0.8, alpha=0.85)
            )
    ax.annotate("", xy=(-W / 2, -L / 2 - 0.14), xytext=(W / 2, -L / 2 - 0.14), arrowprops=dict(arrowstyle="<->", lw=1.0, color=INK))
    ax.text(0, -L / 2 - 0.20, "1100", ha="center", fontsize=8)
    ax.annotate("", xy=(-W / 2 - 0.14, -L / 2), xytext=(-W / 2 - 0.14, L / 2), arrowprops=dict(arrowstyle="<->", lw=1.0, color=INK))
    ax.text(-W / 2 - 0.19, 0, "1620", va="center", rotation=90, fontsize=8)
    ax.annotate("", xy=(0, -WHEELBASE / 2), xytext=(0, WHEELBASE / 2), arrowprops=dict(arrowstyle="<->", lw=0.8, color=ACC, ls="--"))
    ax.text(0.05, 0.30, "轴距 1250", fontsize=7, color=ACC)
    ax.set_xlim(-1.05, 1.05)
    ax.set_ylim(-1.05, 1.05)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("俯视（前向朝上）", fontsize=9)


def side_view(ax):
    ax.add_patch(Rectangle((-L / 2, 0.26), L, 0.66, fc="#f8fafc", ec=INK, lw=1.4))
    ax.add_patch(Rectangle((-BED[0] / 2, 0.92), BED[0], BED[2], fc="#fde68a", ec=ACC, lw=1.1, alpha=0.7))
    ax.text(0, 1.06, "货箱 200 kg", ha="center", fontsize=8, color=ACC)
    for sx in (-1, 1):
        cx = sx * WHEELBASE / 2
        ax.add_patch(Circle((cx, WHEEL_D / 2), WHEEL_D / 2, fc=GREY, ec=INK, lw=0.9, alpha=0.85))
        ax.add_patch(Circle((cx, WHEEL_D / 2), 0.05, fc=INK))
    ax.plot([-L / 2, L / 2], [0.26, 0.26], color=GREY, lw=0.8, ls=":")
    ax.annotate("", xy=(-L / 2 - 0.10, 0.0), xytext=(-L / 2 - 0.10, 0.26), arrowprops=dict(arrowstyle="<->", lw=0.9, color=INK))
    ax.text(-L / 2 - 0.14, 0.12, "260", rotation=90, va="center", fontsize=7)
    ax.annotate("", xy=(L / 2 + 0.06, 1.20), xytext=(L / 2 + 0.06, 0.0), arrowprops=dict(arrowstyle="<->", lw=0.9, color=INK))
    ax.text(L / 2 + 0.10, 0.60, "920", rotation=90, va="center", fontsize=7)
    ax.plot([L / 2 - 0.18, L / 2 - 0.18], [0.92, 1.22], color=INK, lw=2.0)
    ax.add_patch(Rectangle((L / 2 - 0.26, 1.22), 0.16, 0.06, fc="#93c5fd", ec=INK, lw=0.8))
    ax.text(L / 2 - 0.30, 1.31, "LiDAR+相机", fontsize=7, ha="right")
    ax.set_xlim(-1.15, 1.25)
    ax.set_ylim(-0.05, 1.45)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("侧视（前向朝右）", fontsize=9)


def main():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.2, 4.2), dpi=200)
    top_view(ax1)
    side_view(ax2)
    fig.suptitle("SlopeHaul-P1 整机布局：1620×1100×920 mm，4WD 轮毂电机 + 双摆动桥", fontsize=10)
    fig.tight_layout()
    png = OUT / "prototype_layout.png"
    svg = OUT / "prototype_layout.svg"
    fig.savefig(png, metadata={"Software": None}, facecolor="white")
    fig.savefig(svg, metadata={"Date": None, "Creator": None})
    plt.close(fig)
    print("saved", png.relative_to(ROOT), svg.relative_to(ROOT))


if __name__ == "__main__":
    main()
