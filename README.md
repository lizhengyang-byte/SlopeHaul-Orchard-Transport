# SlopeHaul · 山地果园坡地重载运输的地形感知与自适应控制

![SlopeHaul-P1 整机形态（渲染稿）](https://aka.doubaocdn.com/s/A9VVwTwUro)

> 上图为整机形态渲染稿（图像生成，非实拍）；尺寸、质量与能力以本文“整机样机 P1”一节及 `docs/prototype/` 下的核算为准。

面向山地果园"采收后运输"：坡度变化大、道路狭窄、地面松软、路面起伏；负重行驶容易打滑、溜坡、侧倾、停车困难。本项目给出**纯 Python、可一键复现**的地形感知 + 自适应控制闭环，并在五类典型工况下与"定速 + 均分扭矩"基线做同场景对比。

代码量约 3100 行，不依赖 Isaac Sim / ROS / GPU，`python scripts/run_benchmark.py` 即可跑完五个场景 × 两套策略。

除算法外，仓库还包含**整机样机 P1 的可制造方案**（尺寸、爬坡/侧倾/制动/续航核算、电气选型、BOM 3.35 万元、试验大纲）：[`docs/prototype/P1_整机样机方案.md`](docs/prototype/P1_整机样机方案.md)，装配 BOM 见 [`bom_slopehaul_p1.csv`](docs/prototype/bom_slopehaul_p1.csv)。

## 结论先看

| 指标 | 基线（定速+均分扭矩） | 本方案（地形感知+自适应） |
|---|---|---|
| 任务达成（5 场景） | 2/5 | 4/5（其余 1 个为物理不可通行，主动停在坡脚） |
| 平均滑移率 | 10.9% | 0.02% |
| 松软坡爬坡能耗 | 1.064 Wh/m | 0.505 Wh/m（−52.5%） |
| 18° 松软坡驻车 | 后溜 5.21 m | 零溜坡 |

## 它解决什么

| 失效现象 | 机理 | 本项目的做法 |
|---|---|---|
| 负重上坡打滑 | 需求牵引力超过静摩擦能力，轮子空转后 μ 反而下降 | 轮级滑移闭环，只削减扭矩；深空转时整机降扭 |
| 溜坡 | 没比较驻车能力就松刹车 | 驻车状态机先算 `min(制动能力, 静摩擦能力)` 与下滑分力 |
| 侧倾 | 横坡 + 弯道离心力叠加，载荷转移未计入 | 几何包络 `atan(track/2h)`，按裕度收缩限速 |
| 下行刹不住 | 下坡惯性 + 松软地面滚阻低 | 由 `μgcosθ − gsinθ − crrg` 反推允许初速，预瞄限速 |
| 不知道能不能走 | 无地形先验 | 高程平面拟合坡度/粗糙度/台阶 + 土壤分类 + 在线 μ → 可通行代价 |

## 架构

```
高程图(0.15m)  ─▶ 滑窗平面拟合 → 纵坡/横坡/粗糙度/台阶 ┐
地表纹理(32×32)─▶ 8维纹理特征 → MLP 土壤分类 → μ/crr 先验 ├─▶ 可通行性(v_limit/risk/blocked)
IMU/轮速/称重  ─▶ 互补滤波姿态 + 速度EKF + 附着RLS + 载荷┘           │
                                                                     ▼
               安全监督(急停/卡滞回退/降载) ◀─ 稳定包络(侧倾/俯仰) ◀─ 速度规划
                                    │
                                    ▼
              自适应控制：坡度前馈 + 速度PI + 滑移PI + 按载荷分扭矩 + 驻车状态机
                                    ▼
                        四轮扭矩 / 制动 / 驻车指令
```

详细设计见 [`docs/DESIGN.md`](docs/DESIGN.md)，逐场景指标与局限见 [`docs/RESULTS.md`](docs/RESULTS.md)，复刻来源见 [`docs/REFERENCES.md`](docs/REFERENCES.md)。

## 快速开始

```bash
python -m pip install -r requirements.txt
python scripts/run_benchmark.py        # 五场景 × 基线/本方案，写 results/
python scripts/make_figures.py         # 出图到 figures/*.svg
python scripts/run_planner_demo.py     # 代价层 + CoM 规划 + 失效安全 + 滑移模型演示
python scripts/make_prototype_layout.py # 整机布局图（俯视 + 侧视）
python tests/test_smoke.py              # 7 项冒烟测试
python tests/test_planner_failsafe.py   # 新增模块 5 项回归（也可 python -m pytest tests -q）
```

单场景复跑：`python scripts/run_benchmark.py --scenarios S3_hill_hold`；压力测试加 `--noise`（8% 深度点缺失）。

## 五场景（seed=1）

| 场景 | 工况 | 基线 | 本方案 |
|---|---|---|---|
| S1 松软土重载爬坡 | 8°→20°，180 kg，坡中 μ≈0.64 | 未通过，滑移 P95 50%，1.064 Wh/m | **通过** 24.2 s，滑移≈0，0.505 Wh/m |
| S2 横坡弯道 | 14° 横坡 + R≈6 m，200 kg | 通过 24.8 s，侧倾 14.2° | 通过 15.3 s，侧倾 14.8°（包络内） |
| S3 坡道停车再起步 | 18° 松软坡，驻车 8 s | 4.9 s 内失稳，后溜 5.21 m | **通过** 24.5 s，零溜坡 |
| S4 起伏长下坡 | −16°，粗糙度 5.5–7.5 cm | 通过 29.2 s，0.328 Wh/m | 通过 32.0 s，0.336 Wh/m（更保守） |
| S5 湿滑陡坡 | 26°，μ≈0.50，不可通行 | 硬闯后停在 3.2 m，后溜 1.52 m | **主动停在坡脚** 4.1 m，后溜 0.19 m |

图表（地形纵断面、S1 滑移与附着估计、S3 驻车速度、能耗对比）由 `scripts/make_figures.py` 从 `results/` 生成；仓库只提交文本与可复现脚本，不提交生成物与二进制权重。土壤分类权重 `models/soil_mlp.json` 由 `scripts/train_soil_classifier.py` 按固定种子训练得到，`run_benchmark.py` 首次运行会自动补齐。

## v2 增量（不改动上面五场景的任何数字）

| 模块 | 借鉴 | 文件 | 实测 |
|---|---|---|---|
| 2.5D 代价层 | Nav2-Traversability-Layer / terrain_aware_navigation | `perception/costmap.py` | 台阶 0.149 m，同高差在斜坡上仅 0.012 m |
| 重心约束横向规划 | AgRobPP-CA | `planning/lateral_planner.py` | 中线 56 行被台阶占住，规划全程不进禁行区，代价降 94% |
| 失效安全阶梯 | ArduPilot ArduRover | `control/failsafe.py` | 链路中断→HOLD、过温→DERATE、急停→STOP 并锁存 |
| 坡度/速度滑移模型 | Frontiers 2025 载货滑移模型 | `estimation/load_slip.py` | S1+S3 遥测标定 R² 0.59、MAE 0.059 |

演示地形为 14° 上坡窄道 + 右侧 0.18 m 台阶，数字见 `results/planner_demo.json`。为何滑移模型里没有“载荷项”（质量是比率量，会约掉）、哪些开源项目**没有**被采用（ROS/Gazebo 栈、METAVerse、TartanDrive 等），见 `docs/prototype/P1_整机样机方案.md` 第 6 节。

## 整机样机 P1（可制造级方案）

| 项 | 指标 |
|---|---|
| 尺寸 / 轴距·轮距 / 轮径 | 1620×1100×920 mm / 1250·1100 / 480 mm |
| 整备 + 额定载重 | 320 + 200 kg（最大 250 kg） |
| 行走 | 4×4 48 V 轮毂电机 + 前后两组被动摆动桥（±12°） |
| 爬坡 / 侧倾 / 续航 | 干硬 25°、松软 20°、湿滑 18° / 动态包络 31.9° / 3.8 km（满载坡） |
| 制动 | 前液压盘刹 + 后电磁失电制动，均 2×160 N·m，断电即驻 |
| BOM | 单台 3.35 万元（含 GNSS-RTK 选配 3.61 万元） |

与开源样机的取舍写得比较直：Agrofelis（MIT）只取指标基线，因其无悬挂且 992 mm 宽不适合窄行；AgriCruiser 为 GPL v3 不复制文件；AgriBot 无 LICENSE 不引用代码；Papaya 的摇臂原理取用但改四轮；ArduRover 只对齐失效安全语义。

## 制造交付包（可直接下厂的 v1）

图纸与文档不是 PPT 里的示意图，是能直接发给工厂的图册 + 判据 + 排期。

| 交付物 | 位置 | 说明 |
|---|---|---|
| 机械图纸 4 张 | `cad/out/SLH-P1-01…04.dxf` | 车架 / 摆动桥 / 货箱 / 总装，A3，1:10 |
| 电气图纸 2 张 | `cad/out/SLH-P1-E01/E02.dxf` | 主回路（48V→4×轮毂 + DC-DC）、控制拓扑（Jetson↔H7↔传感器） |
| 线束表 | `cad/out/harness_slopehaul_p1.csv` | J1–J12，针脚/线径/长度 |
| 图纸清单与工艺 | `docs/manufacturing/01_*.md` | 公差表、工艺路线、外购件询价要点 |
| 检验与验收 | `docs/manufacturing/02_*.md` | IQC/IPQC/FAT 三档判据，16 项出厂试验 |
| 试制与交付清单 | `docs/manufacturing/03_*.md` | 27 人日、15 工作日排期、随机交付物、风险表 |
| 出图脚本 | `cad/build_drawings.py`、`cad/build_electrical.py` | 图纸唯一真源，改脚本重出，不手改 DXF |

```bash
pip install ezdxf matplotlib
python -m cad.build_drawings     # 机械 4 张：DXF + PDF + PNG
python -m cad.build_electrical   # 电气 2 张 + 线束表
```

部署侧代码 `slopehaul/deploy/` 已给出上车可用的最小闭环：CAN 应用层协议（0x180/0x200，CRC-16/CCITT）、H7 侧 200 Hz 监督器（心跳 200 ms 看门狗、扭矩斜率限制、急停锁存）、Jetson 侧 20 Hz 定频节点（含 deadline miss 统计），12 项单测随 `pytest tests/test_deploy.py` 跑通。

两处必须提前知道的边界：图纸模型空间是**纸张坐标**（1:10），量图元要 ×10，标注写的是真实值；三维 STEP 装配尚未做（本机无 cadquery/freecad），二维 DXF 足以开料与焊接，但管路与干涉仍需三维复核。这两条已写进 `cad/README.md` 与 `docs/manufacturing/03` 的 v1.1 计划。

## 复刻来源（重要）

参考三个开源项目的思路做**可运行复刻**，不是搬代码：

1. [`PercyGeza/OrchardVLA-Mountain-Orchard-Robot`](https://github.com/PercyGeza/OrchardVLA-Mountain-Orchard-Robot) —— 复刻"分层仿真 + 感知→规划→控制→安全监督"的分层与安全监督设计。原仓库依赖 Isaac Sim / Isaac Lab，本机无法运行，故改用纯 Python 轻量仿真承载同样的分层。
2. [`rpl-cmu/rose`](https://github.com/rpl-cmu/rose) —— 复刻"轮速不可信时用 IMU 预测、按残差自适应门限更新"的思路，落成 `VelocityEKF` + `FrictionRLS`。
3. [`Vishnusathvik/Fast-Lio_traversibility_mapper`](https://github.com/Vishnusathvik/Fast-Lio_traversibility_mapper) —— 复刻"高程 → 坡度/粗糙度/台阶 + 足迹膨胀 → 可通行代价"的建图思路。

本仓库代码为独立实现（MIT），未复制上述仓库源码。相对它们的增量是**重载耦合**（轴荷转移、质心随载重升高、软土承载折减）、**驻车能力可计算**、**下行制动距离反推限速**、**能力边界显式化**。

## 目录

```
slopehaul/
  config.py            车辆/感知/控制/场景参数
  sim/                 地形、传感器、车辆动力学、闭环世界
  perception/          高程特征、土壤分类(MLP)、可通行性、2.5D 代价层
  estimation/          速度EKF、附着RLS、滑移率、载荷估计、坡度滑移代理模型
  planning/            代价层上的横向偏移规划（重心投影约束）
  control/             自适应控制、稳定性包络、驻车状态机、安全监督、失效安全阶梯
  agents/transporter.py感知-决策-控制串联，含 baseline/proposed 两套策略
  eval/                五场景、指标、批量评测
cad/                   图纸生成脚本（机械 4 张 + 电气 2 张 + 线束表），产物在 cad/out
scripts/               跑批 / 训练 / 出图 / 规划演示 / 整机布局图
models/                土壤分类权重（JSON，可由脚本复现）
docs/                  设计说明、结果解读、参考项目
  prototype/           整机样机 P1 方案与 BOM
  manufacturing/       图纸清单与工艺、检验与验收、试制与交付清单
tests/                 冒烟回归 + 新增模块回归
```

## 诚实的局限

- 物理模型是"纵向单轨 + 准静态侧倾/俯仰"的集总参数模型，没做多体动力学与轮胎-土壤离散元耦合；结论适用于**控制策略层面的相对比较**，不能替代实车标定。
- 土壤分类纹理是**合成代理数据**，验证集 100% 只说明四类在当前合成分布下可分，**不代表真实果园影像精度**。
- 静止或低滑移时附着系数几乎不可观测（S1 的 μ 估计 MAE 0.224 即由此而来），此时只给先验区间，不谎报精确值。
- 场景参数按山地果园常见工况设定，尚未用实测地形数据标定。
- 整机样机的 BOM 价格为 2026-11 询价口径，批量采购可降约 15%，但**未做样机实测**，所有能力值均来自核算与仿真。

## 许可

MIT，见 [LICENSE](LICENSE)。
