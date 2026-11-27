# SlopeHaul · 山地果园坡地重载运输的地形感知与自适应控制

面向山地丘阾果园"采掳后运输"环节：坡度变化大、路窄、地面松软、路面起伏，负重行驶易打滑、溜坡、侧倾、停车困难。本项目给出一套**纯 Python、可一键复现**的地形感知 + 自适应控制闭环，并在五类典型工况下与"定速 + 均分扭矩"基线做同场景对比。

## 一句话结论

在五个预置工况里，基线只有 2 个能到终点，本方案 4 个到达（第 5 个是物理不可通行的湿滑陡坡，本方案主动停在坡脚而不是硬闯）；平均滑移率从 **10.9% 降到 0.02%**，松软土爬坡场景单位里程能耗下降 **52.5%**，18° 松软坡驻车由"后溜 5.2 m"变为"零溜坡"。

## 它解决什么

| 现象 | 机理 | 本项目的做法 |
|---|---|---|
| 负重上坡打滑 | 需求牵引力超过地面静摩擦能力，轮子空转后 μ 反而下降 | 轮级滑移闭环，只削减扭矩不加码；深空转时整机降扭 |
| 溜坡 | 制动/驻车力未与坡道下滑分力比较就松刹车 | 驻车状态机先算 `min(制动能力, 静摩擦能力)`，不足则不释放 |
| 侧倾 | 横坡 + 弯道离心力叠加，载荷转移没被计入 | 几何推导侧倾/俯仰包络（`atan(track/2h)`），裕度收缩限速 |
| 停车困难 | 下坡惯性 + 松软地面滚阻低 | 依据 `μgcosθ − gsinθ − crrg` 反推允许制动初速，预瞄限速 |
| 不知道路面能不能走 | 没有地形先验，全靠人眼 | 高程平面拟合出坡度/粗糙度/台阶，融合视觉土壤分类与在线 μ，输出可通行代价 |

## 系统架构

```
                    ┌──────────────────── 感知层 ────────────────────┐
 高程图(0.15m栅格) ─▶│ 滑窗平面拟合 → 纵坡/横坡/粗糙度/台阶        │
 地表纹理(32×32)  ─▶│ 纹理特征(8维) → MLP 土壤分类 → μ/crr 先验     │
 IMU / 轮速 / 称重 ─▶│ 互补滤波姿态、EKF 车速、RLS 在线附着、载荷估计│
                    └───────────────────┬───────────────────────────┘
                                        ▼
                    可通行性评估（坡度上限、粗糙度、台阶、足迹膨胀、
                    制动距离 → v_limit / risk / cost）
                                        ▼
        稳定性包络（侧倾/俯仰裕度 → 速度缩放） ──▶ 安全监督（急停/卡滞回退/降载）
                                        ▼
        自适应控制（坡度前馈 + 速度 PI + 滑移 PI + 载荷分配 + 驻车状态机）
                                        ▼
                        四轮扭矩 / 制动 / 驻车指令
```

模块与文件一一对应，见下方目录结构；详细设计见 [`docs/DESIGN.md`](docs/DESIGN.md)，指标明细见 [`docs/RESULTS.md`](docs/RESULTS.md)。

## 快速开始

```bash
python -m pip install -r requirements.txt
python scripts/run_benchmark.py            # 五场景 × 基线/本方案，写 results/
python scripts/make_figures.py             # 生成 figures/*.svg
python tests/test_smoke.py                 # 免 pytest 的冒烟测试
```

单场景复跑：

```bash
python scripts/run_benchmark.py --scenarios S3_hill_hold
```

## 五场景结果（seed=1，`results/comparison.md` 为原始表）

| 场景 | 工况 | 基线 | 本方案 |
|---|---|---|---|
| S1 松软土重载爬坡 | 8°→20°，载重 180 kg，坡中 μ≈0.64 | 未通过，滑移 P95 50%，1.064 Wh/m | **通过**，24.2 s，滑移≈0，0.505 Wh/m |
| S2 横坡弯道 | 14° 横坡 + R≈6 m 弯，满载 200 kg | 通过，24.8 s，侧倾 14.2° | 通过，15.3 s，侧倾 14.8°（包络内） |
| S3 坡道停车再起步 | 18° 松软坡，中途驻车 8 s | 4.9 s 内失稳，后溜 5.21 m | **通过**，24.5 s，零溜坡 |
| S4 起伏长下坡 | −16°，粗糙度 5.5–7.5 cm | 通过，29.2 s，0.328 Wh/m | 通过，32.0 s，0.336 Wh/m（更保守） |
| S5 湿滑陡坡 | 26°，μ≈0.50，物理不可通行 | 硬闯后停在 3.2 m，后溜 1.52 m | **主动拒绞**，停 4.1 m，后溜 0.19 m |

![地形纵断面](figures/figures_terrain_profiles.svg)

![S1 滑移与附着估计](figures/figures_s1_slip.svg)

![S3 驻车对比](figures/figures_s3_hillhold.svg)

![能耗对比](figures/figures_energy_compare.svg)

## 复刻来源与差异（重要）

本项目是**参考三个开源项目的思路做的可运行复刻**，不是把它们的代码直接搬过来跑：

1. [`PercyGeza/OrchardVLA-Mountain-Orchard-Robot`](https://github.com/PercyGeza/OrchardVLA-Mountain-Orchard-Robot)
   复刻其"分层仿真 + 感知→规划→控制→安全监督"的系统分层与安全监督设计（心跳、急停、距离限速、命令裁剪）。
   **差异**：原仓库依赖 Isaac Sim / Isaac Lab，本机无法运行；本项目改用纯 Python 的轻量物理与传感器仿真，把同样的分层落到可复现的 CPU 环境。
2. [`rpl-cmu/rose`](https://github.com/rpl-cmu/rose)（Robust Off-road wheel odometry with Slip Estimation）
   复刻其"轮速不可信时用 IMU 预测、按残差自适应门限更新"的思路，落成 `VelocityEKF` + `FrictionRLS`。
3. [`Vishnusathvik/Fast-Lio_traversibility_mapper`](https://github.com/Vishnusathvik/Fast-Lio_traversibility_mapper)
   复刻其"高程 → 坡度/粗糙度/台阶 + 车辆足迹膨胀 → 可通行代价"的建图思路，落成 `perception/elevation.py`。

三个仓库均为参考与致谢对象；本仓库代码为独立实现（MIT），未复制其源码，许可不冲突。详见 [`docs/REFERENCES.md`](docs/REFERENCES.md)。

## 目录

```
slopehaul/
  config.py            # 车辆/感知/控制/场景参数中心
  sim/                 # 地形、传感器、车辆动力学、闭环世界
  perception/          # 高程特征、土壤分类(MLP)、可通行性、姿态融合
  estimation/          # 速度EKF、附着RLS、滑移率、载荷估计
  control/             # 自适应控制、稳定性包络、驻车状态机、安全监督
  agents/transporter.py# 感知-决策-控制串联，含 baseline/proposed 两套策略
  eval/                # 五场景、指标、批量评测
scripts/               # 一键跑批、训练、出图
models/soil_mlp.json   # 8-16-4 土壤分类权重（文本 JSON，合成纹理训练）
results/               # summary.json / telemetry_*.csv / comparison.md
figures/               # 报告用 SVG
docs/                  # 设计说明、结果解读、参考文献
tests/                 # 冒烟回归
```

## 诚实的局限

- 物理模型是**单轨纵向 + 准静态侧倾/俯仰**的集总参数模型，没有做多体动力学与轮胎-土壤离散元耦合；结论适用于"控制策略层面的相对比较"，不能替代实车标定。
- 土壤分类的纹理是**合成代理数据（proxy）**，验证集 100% 只说明四类在当前合成分布下可分，**不代表真实果园影像精度**；上线前需用真实相机数据重标。
- 静止或低滑移时附着系数几乎不可观测（S1 的 μ 估计 MAE 0.224 即由此而来），此时系统只给土壤先验区间而不谎报精确值。
- 场景参数（坡度、μ、载重）是按丘阾果园常见工况设定的代表值，尚未用实测地形数据标定。

## 许可与致谢

代码 MIT。参考仓库的作者与项目见 `docs/REFERENCES.md`，仅作思路与方法引用。
