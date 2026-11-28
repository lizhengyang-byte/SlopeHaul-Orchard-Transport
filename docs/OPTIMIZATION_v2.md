# v2 优化：向开源生态对标后补了四个模块

这一轮不改写已冻结的 S1–S5 结论（数字一个不动），而是把“别人的成熟做法”拆成四个可独立验证的模块补进来，各自有测试、有演示、有可核对的数字。

## 1. 借鉴关系（做了什么、取自哪里、落在哪个文件）

| 参考项目 | 借鉴的具体做法 | 落成位置 | 解决命题的哪一面 |
|---|---|---|---|
| [Nav2-Traversability-Layer](https://github.com/abulhasnat-abdullah/Nav2-Traversability-Layer) | 坡度、台阶、粗糙度三类特征 → 代价层；**台阶判定做坡度校正**，区分“石头”和“能上的坡” | `perception/costmap.py` | 路面起伏、台阶误判 |
| [terrain_aware_navigation](https://github.com/MOHANsundar007/terrain_aware_navigation) | grid_map 式 2.5D 高程层 + 自定义代价图层，供规划器直接消费 | `perception/costmap.py` | 地形感知 → 规划接口 |
| [AgRobPP / AgRobPP-CA](https://gitlab.inesctec.pt/agrob/agrob_pp/) | 规划显式考虑车体朝向与**重心投影**；局部高程图估计倾角与危险坡度 | `planning/lateral_planner.py` | 负重侧倾、窄道通行 |
| [costmap-uncertainty](https://github.com/TalayKD/costmap-uncertainty) | 对不确定/不可通行区域做膨胀，避免贴着危险边缘走 | `perception/costmap.py` | 松软地面边界不可信 |
| [ArduPilot ArduRover](https://ardupilot.org/rover/) | 失控保护分层：链路丢失、地理围栏、姿态超限、电机过温、外部急停；急停需显式复位 | `control/failsafe.py` | 溜坡、停车难、无人看管作业 |
| Frontiers 2025 载货滑移模型 | 用坡度、速度、附着解释滑移率，做可离线外推的代理模型 | `estimation/load_slip.py` | 打滑预测与重载决策 |
| [Agrofelis](https://github.com/meltoner/agrofelis) | 250 kg 级“AI-ready”运载底盘，轮毂电机实时反馈调速 | 参数对标（本车 320+200 kg） | 整机定位与指标对照 |

## 2. 四个模块各自的效果（数字来自 `results/planner_demo.json`）

演示地形：14° 上坡窄道（可走宽度约 1.1 m），右侧 s≥6 m 处有 0.18 m 台阶。

**① 代价层（坡度校正的台阶判据）**

- 台阶处读数 0.149 m，同一高差摊在斜坡上只有 0.012 m —— 说明“斜坡正常爬升”被扣掉，不会像第一版那样把整条下坡判死（v1 曾因 0.42 m 波长起伏混叠出 0.21 m 假台阶）。
- 窄道上不可通行栅格占比 22.7%，全部集中在台阶及其膨胀区。

**② 重心约束横向规划**

- 中线（y=0）有 56 行落在禁行区，直接走中线等于撞台阶；
- 规划轨迹全程不进入禁行栅格，最差安全余量 0.15 m，代价相对中线降低 94%；
- 横向偏移的逐行变化率均值 0.011 m/m，说明规划是“平顺绕行”而不是左右横跳。

**③ 失效安全阶梯**

15 s 时间线里依次触发：6 s 链路中断 → HOLD；8 s 链路恢复 → OK；10 s 电机 78 ℃ → DERATE（限速 0.7）；12 s 外部急停 → STOP 并**锁存**；撤销急停信号后仍保持 STOP，只有操作员 `clear_estop(ack=True)` 才回到 OK。这条“急停必须人工复位”的规则直接照搬工业底盘的惯例。

**④ 载荷/坡度滑移代理模型**

用 S1、S3 的遥测（188 个采样点）标定，R² = 0.59、MAE = 0.059；坡度每增加 1° 滑移率增加约 0.7 个百分点。

一个必须写明的物理事实：滑移率是**比率量**，质量在“牵引需求”和“附着能力”里同时出现会相互约掉，所以模型里没有单独的载荷项——载荷是通过质心高度、轴荷转移、软土承载折减进入车辆模型的（见 `docs/DESIGN.md` 4.3）。强行给载荷一个系数只会拟合出负值，不如不给。

## 3. 明确没有采用的东西（以及理由）

| 项目 | 不采用的原因 |
|---|---|
| AgRobPP 的 ROS 1 + Gazebo 葡萄园仿真 | 依赖 ROS/Gazebo 全栈，纯 CPU 环境不可复现；且代码托管在 GitLab，许可需单独确认 |
| ROS 2 + Nav2 + vox_nav 2.5D 栈（botanbot_sim、orchard_simulation 等） | 同上：能跑起来但结果不可在 CI 里逐位复现，与本项目“一键复现”的取舍冲突 |
| METAVerse（元学习代价图） | 论文开源了方法但截至检索未见公开代码，只能作为路线图 |
| TartanDrive / RELLIS-3D / CaT 数据集 | 数据体量大（数十 GB）且为真实越野场景，本项目的合成代理地形用不上；列为后续标定数据源 |
| ArduRover 固件本身 | 目标是“算法层可验证”，不涉及飞控/固件移植；只借其保护逻辑分层 |

## 4. 下一步（有明确接口，不是口号）

1. **代价层对接 ROS 2**：`perception/costmap.py` 输出已是 `(s, y) → cost/blocked` 的标准栅格，可直接转成 `nav2_costmap_2d` 的 `CostmapLayer` 插件，供 Nav2 规划器消费。
2. **规划层换 A\***：当前是逐行动态规划（窄道足够），换成 AgRobPP 那类带朝向与 CoM 约束的 A\* 后可直接处理分叉与绕行。
3. **滑移模型换数据驱动**：接入 TartanDrive 的动力学标注，把现在的三项线性代理模型换成小型网络，并把“在线自适应”接回 `FrictionRLS`。
4. **执行层对齐 ArduRover**：把 `control/failsafe.py` 的四级状态映射成 ArduRover 的 `FS_*` 参数语义，便于真机部署时直接搬。
