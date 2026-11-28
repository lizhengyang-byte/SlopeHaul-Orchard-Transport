# GitHub 同步说明（仓库收录范围与生成物边界）

本仓库只提交**可审阅的文本与可复现的脚本**，不提交体积大、可一键重出的生成物。这样做的目的是：克隆下来 10 秒内能看懂、能跑，而不是被几十兆二进制拖死。

## 1. 已入库（随 commit 版本受控）

| 类别 | 内容 |
|---|---|
| 算法层 | `slopehaul/`（perception / estimation / planning / control / eval / sim） |
| 部署层 | `slopehaul/deploy/`：`can_protocol.py`、`supervisor.py`、`jetson_node.py` |
| 测试 | `tests/`（24 项，`python -m pytest tests -q` 全绿） |
| 出图脚本 | `cad/build_drawings.py`、`cad/build_electrical.py`、`cad/dwg_common.py`、`cad/README.md` |
| 制造文档 | `docs/manufacturing/01–03`（图纸清单与工艺、检验与验收、试制与交付） |
| 整机方案 | `docs/prototype/01–05`（总体、机械、电气 BOM、控制软件、装配试验） |
| 分析与对比 | `docs/OPTIMIZATION_v2.md`、`results/comparison.md` |
| 工程说明 | `README.md`、`docs/DESIGN.md`、`docs/REFERENCES.md` |
| CI | `.github/workflows/verify.yml`：测试 + 重跑基准 + 重出图 + 上传 artifacts |

## 2. 不入库（生成物 / 二进制），以及在哪里拿到

| 文件 | 体积 | 重出命令 | 获取位置 |
|---|---|---|---|
| `cad/out/*.dxf/.pdf/.png`（6 图 3 格式） | 合计 ~2.5 MB | `python -m cad.build_drawings` / `python -m cad.build_electrical` | CI artifacts `slopehaul-generated`；飞书「06 制造交付包」 |
| `cad/out/harness_slopehaul_p1.csv` | 3 KB | 同上（含在出图脚本里） | CI artifacts；飞书同上 |
| `figures/*.svg`（6 张图表 + 布局图） | 合计 ~300 KB | `python -m scripts.make_figures` / `make_prototype_layout` | CI artifacts；README 内联图另有 CDN 版本 |
| `results/*.json`、`telemetry_S*.csv` | 合计 ~700 KB | `python -m scripts.run_benchmark` | CI artifacts |
| `models/soil_mlp.json`（土壤分类权重） | 2.5 KB | `python -m scripts.train_soil_classifier` | CI artifacts |

理由三条：一是体积（遥测 CSV 单文件最大 204 KB）；二是二进制无法在 PR 里 review；三是所有生成物都由**固定 seed** 的脚本产出，CI 每次 push 都会重跑一遍，数字不靠口头承诺。

## 3. 一致性怎么保证

- CI 在每次 push / PR 上跑 `pytest` + `run_benchmark`，测试失败则红灯；
- 图纸脚本内置坐标与尺寸断言（A3 图框、标题栏位置、缩放比例），改标尺会直接报错；
- 文档里的每个数字都能在 `results/summary.json` 或本文表格里找到出处，CII artifacts 可下载后对账。

## 4. 与飞书归档的关系

飞书「06 制造交付包」是**面向工厂的二进制权威副本**（6 DXF + 6 PDF + 6 PNG + 线束表 + 3 篇 docx）；本仓库是**源码与文本权威副本**。两边内容来自同一份脚本输出，时间戳差来自重跑，数值一致。
