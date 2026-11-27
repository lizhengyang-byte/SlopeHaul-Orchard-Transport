# SlopeHaul · 山地果园坡地重载运输的地形感知与自适应控制

面向山地丘阾果园"采掳后运输"：坡度变化大、道路狭窄、地面松软、路面起伏；负重行驶容易打滑、溜坡、侧倾、停车困难。本项目给出**纯 Python、可一键复现**的地形感知 + 自适应控制闭环，并在五类典型工况下与"定速 + 均分扭矩"基线做同场景对比。

代码量约 2600 行，不依赖 Isaac Sim / ROS / GPU，`python scripts/run_benchmark.py` 即可跑完五个场景 × 两套策略。

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
python scripts/run_benchmark.py      # 五场景 × 基线/本方案，写 results/
python scripts/make_figures.py       # 出图到 figures/*.svg
python tests/test_smoke.py           # 7 项冒烟测试（也可 python -m pytest tests -q）
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
  perception/          高程特征、土壤分类(MLP)、可通行性、姿态融合
  estimation/          速度EKF、附着RLS、滑移率、载荷估计
  control/             自适应控制、稳定性包络、驻车状态机、安全监督
  agents/transporter.py感知-决策-控制串联，含 baseline/proposed 两套策略
  eval/                五场景、指标、批量评测
scripts/               跑批 / 训练 / 出图
models/                土壤分类权重（JSON，可由脚本复现）
docs/                  设计说明、结果解读、参考项目
tests/                 冒烟回归
```

## 诚实的局限

- 物理模型是"纵向单轨 + 准静态侧倾/俯仰"的集总参数模型，没做多体动力学与轮胎-土壤离散元耦合；结论适用于**控制策略层面的相对比较**，不能替代实车标定。
- 土壤分类纹理是**合成代理数据**，验证集 100% 只说明四类在当前合成分布下可分，**不代表真实果园影像精度**。
- 静止或低滑移时附着系数几乎不可观测（S1 的 μ 估计 MAE 0.224 即由此而来），此时只给先验区间，不谎报精确值。
- 场景参数按丘阾果园常见工况设定，尚未用实测地形数据标定。

## 许可

MIT，见 [LICENSE](LICENSE)。
