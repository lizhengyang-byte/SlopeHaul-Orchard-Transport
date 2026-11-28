# cad/ 图纸生成说明

二维图纸全部由脚本生成，DXF 是产物不是源文件。改图请改脚本，重跑即可，避免"图纸和实物对不上"。

## 依赖

```bash
pip install ezdxf matplotlib
```

## 出图

```bash
python -m cad.build_drawings      # 机械 4 张：DXF + PDF + PNG
python -m cad.build_electrical    # 电气 2 张 + 线束表 CSV
```

输出在 `cad/out/`：

```
SLH-P1-01_FRAME ASSY.dxf/.pdf/.png
SLH-P1-02_SWING BOGIE.dxf/.pdf/.png
SLH-P1-03_CARGO BOX.dxf/.pdf/.png
SLH-P1-04_GEN ARRANGEMENT.dxf/.pdf/.png
SLH-P1-E01_POWER CIRCUIT.dxf/.pdf/.png
SLH-P1-E02_CONTROL TOPOLOGY.dxf/.pdf/.png
harness_slopehaul_p1.csv
```

## 约定

| 项 | 取值 |
|---|---|
| 图幅 | A3 420×297，图框内缩 8 mm |
| 标题栏 | 182×40，位于 (230, 8)，4 行 |
| 文字高度 | 标注 3.5 mm，正文 3.0 mm，小字 2.0–2.5 mm |
| 线宽/颜色 | 按图层：OUTLINE/HIDDEN/CENTER/DIM/TEXT/TITLE/HOLES/WIRE/POWER/GND/SIGNAL |
| 比例 | 零件按 1:10 落纸（局部 1:5、2:1），电气图不按比例 |
| 单位 | `$INSUNITS = 4`（mm） |

**注意**：模型空间是纸张坐标，图元长度是真实尺寸的 1/10，标注文字写真实值。在 CAD 里量图元需 ×10，或直接读标注。

## 已知限制

- 只有二维 DXF，没有 STEP/三维装配（本机无 cadquery/freecad，未做）。
- 未使用布局视口，1:1 模型空间出图列为 v1.1。
- 中文标注在 DXF 里用宋体（`simsun.ttc`）；无该字体的机器上 DXF 仍能打开，但预览图由 matplotlib 渲染，故图面文字统一用英文 + 尺寸数字，避免方块字。

## 新增一张图

1. 在 `build_drawings.py` 里加一个 `draw_xxx(msp)`，用 `cad.dwg_common` 的 `rect/txt/dim_h/dim_v`；
2. 在 `main()` 的 `made` 列表里加 `build("名称", "SLH-P1-05", "材料", draw_xxx, [注记])`；
3. 图号沿用 `SLH-P1-NN`，标题栏会自动带版本与日期。
