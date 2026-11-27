# 参考项目与复刻边界

本项目的做法是「借鉴架构、重写实现、针对坡地重载做增量」，而不是 fork 某个仓库。三个参考项目的能力被拆开吸收，各自贡献一块。

## 1. PercyGeza/OrchardVLA-Mountain-Orchard-Robot

链接：https://github.com/PercyGeza/OrchardVLA-Mountain-Orchard-Robot

参考内容：

- 分层仿真结构：感知 → 规划 → 控制 → 安全监督，每层职责单一、可单独测试；
- 安全监督的位置：不嵌在控制器里，而是作为最后一层对指令做约束与拦截（心跳、急停、距离限速、命令裁剪）；
- 果园场景的任务组织方式（多阶段作业、坡地行走）。

没有直接复用其代码的原因：

- 依赖 Isaac Sim 5.1 与 Isaac Lab 2.3.2，需要 GPU 与数十 GB 环境，本机无法运行；
- 许可为 PolyForm Noncommercial，商用与竞赛再分发需另行判断；
- 本项目要验证的是“感知与控制的闭环逻辑”，不需要高保真渲染。

于是改用纯 Python 轻量仿真承载同一套分层，保证任何人 `pip install -r requirements.txt` 后能直接复现结果。

## 2. rpl-cmu/rose

链接：https://github.com/rpl-cmu/rose

参考内容：

- 轮速在打滑时不可信，需要 IMU 做预测、轮速做带门限的更新；
- 残差自适应门限：残差大就降权，而不是直接丢弃量测。

落成实现：`slopehaul/estimation/slip.py` 的 `VelocityEKF`（状态 `[v, b]`，IMU 预测 + 轮速/视觉自适应门限更新）与 `FrictionRLS`（标量遗忘最小二乘，带滑移窗口与限幅）。

差异：原项目面向轮式机器人在多地形上的里程计与地形分类，本项目把同一思想用在“负重坡地运输车”的滑移与附着估计上，并加了与滑移闭环的耦合（估计结果直接决定扭矩削减量）。

## 3. Vishnusathvik/Fast-Lio_traversibility_mapper

链接：https://github.com/Vishnusathvik/Fast-Lio_traversibility_mapper

参考内容：

- 从高程图算坡度、粗糙度、台阶三类几何特征；
- 足迹膨胀：按车辆尺寸膨胀不可通行区域，得到可通行代价地图。

落成实现：`slopehaul/perception/{elevation,traversability,fusion}.py`。台阶判据从“窗口内极差”改成“相邻栅格最大高差”，避免长波起伏被当成台阶（这是踩过的坑，详见 docs/DESIGN.md）。

差异：原项目依赖 FAST-LIO 激光惯性里程计与 ROS，本项目用解析地形直接生成高程图，把可通行性判定与制动距离反推、载重惩罚结合，输出的是速度上限而不只是代价地图。

## 4. 检索过但未采用的项目

| 仓库 | 未采用原因 |
|---|---|
| mertcetinturk/autonomous-orchard-robot | 偏 ROS2 导航栈搭建，缺少坡地与附着建模，无法支撑“打滑/溜坡”命题核心 |
| 1344618323/OrchardRover | 偏机械结构与底盘，感知控制部分薄弱 |
| TUMFTM/3DVehicleDynamicsStateEstimation | 车辆状态估计很扎实，但没有地形感知与可通行性判定，且偏乘用车 |

## 5. 相对参考项目的增量

1. **负重与质心变化进模型**：载荷估计驱动质心高、轴荷转移、稳定性包络，参考项目多为空载或固定载荷假设。
2. **驻车能力判定**：比较制动能力、静摩擦能力与坡道下滑分力，不足则报 slipping 并累计滑距，而不是默认停得住。
3. **统一的五场景基准**：同一套指标下对比开环基线与自适应方案，能直接看出“哪一项能力换来了什么”。
4. **零重依赖可复现**：只依赖 numpy 与 matplotlib，权重缺失自动训练，结果单种子可逐位复现。

## 6. 许可声明

本项目未复制上述仓库的源代码，只借鉴其架构思路与算法思想，全部实现为重写。本仓库代码以 MIT 许可发布；若后续引入参考项目的任何文件，需按其原许可（如 PolyForm Noncommercial）单独标注。
