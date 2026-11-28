# 基线与本方案对比

| 场景 | 指标 | 基线（定速+均分扭矩） | 本方案（地形感知+自适应） |
|---|---|---|---|
| S1_soft_climb | 任务结论 | 任务失败 | 到达终点 |
| S1_soft_climb | 通过场景 | 否 | 是 |
| S1_soft_climb | 耗时 (s) | 40.0 | 24.2 |
| S1_soft_climb | 平均滑移率 | 9.0% | 0.0% |
| S1_soft_climb | P95 滑移率 | 50.0% | 0.0% |
| S1_soft_climb | 严重打滑占比 | 12.7% | 0.0% |
| S1_soft_climb | 最大侧倾角 (°) | 2.0 | 2.2 |
| S1_soft_climb | 倾翻 | 否 | 否 |
| S1_soft_climb | 后溜距离 (m) | 1.94 | 0.00 |
| S1_soft_climb | 驻车溜坡 (m) | 0.00 | 0.00 |
| S1_soft_climb | 附着估计 MAE | 0.128 | 0.224 |
| S1_soft_climb | 能耗 (Wh/m) | 1.064 | 0.505 |
| S1_soft_climb | 安全介入 | 0 | 8 |
| S2_cross_slope | 任务结论 | 到达终点 | 到达终点 |
| S2_cross_slope | 通过场景 | 是 | 是 |
| S2_cross_slope | 耗时 (s) | 24.8 | 15.3 |
| S2_cross_slope | 平均滑移率 | 0.0% | 0.0% |
| S2_cross_slope | P95 滑移率 | 0.0% | 0.0% |
| S2_cross_slope | 严重打滑占比 | 0.0% | 0.0% |
| S2_cross_slope | 最大侧倾角 (°) | 14.2 | 14.8 |
| S2_cross_slope | 倾翻 | 否 | 否 |
| S2_cross_slope | 后溜距离 (m) | 0.00 | 0.00 |
| S2_cross_slope | 驻车溜坡 (m) | 0.00 | 0.00 |
| S2_cross_slope | 附着估计 MAE | 0.015 | 0.015 |
| S2_cross_slope | 能耗 (Wh/m) | 0.321 | 0.294 |
| S2_cross_slope | 安全介入 | 0 | 0 |
| S3_hill_hold | 任务结论 | 任务失败 | 到达终点 |
| S3_hill_hold | 通过场景 | 否 | 是 |
| S3_hill_hold | 耗时 (s) | 4.9 | 24.5 |
| S3_hill_hold | 平均滑移率 | 43.4% | 0.1% |
| S3_hill_hold | P95 滑移率 | 50.0% | 0.6% |
| S3_hill_hold | 严重打滑占比 | 81.6% | 0.0% |
| S3_hill_hold | 最大侧倾角 (°) | 3.0 | 3.0 |
| S3_hill_hold | 倾翻 | 否 | 否 |
| S3_hill_hold | 后溜距离 (m) | 5.21 | 0.00 |
| S3_hill_hold | 驻车溜坡 (m) | 0.00 | 0.00 |
| S3_hill_hold | 附着估计 MAE | 0.054 | 0.035 |
| S3_hill_hold | 能耗 (Wh/m) | 5.694 | 0.651 |
| S3_hill_hold | 安全介入 | 1 | 1 |
| S4_rough_downhill | 任务结论 | 到达终点 | 到达终点 |
| S4_rough_downhill | 通过场景 | 是 | 是 |
| S4_rough_downhill | 耗时 (s) | 29.2 | 32.0 |
| S4_rough_downhill | 平均滑移率 | 0.0% | 0.0% |
| S4_rough_downhill | P95 滑移率 | 0.0% | 0.0% |
| S4_rough_downhill | 严重打滑占比 | 0.0% | 0.0% |
| S4_rough_downhill | 最大侧倾角 (°) | 4.0 | 4.0 |
| S4_rough_downhill | 倾翻 | 否 | 否 |
| S4_rough_downhill | 后溜距离 (m) | 0.00 | 0.00 |
| S4_rough_downhill | 驻车溜坡 (m) | 0.00 | 0.00 |
| S4_rough_downhill | 附着估计 MAE | 0.044 | 0.044 |
| S4_rough_downhill | 能耗 (Wh/m) | 0.328 | 0.336 |
| S4_rough_downhill | 安全介入 | 0 | 0 |
| S5_steep_wet | 任务结论 | 安全拒绝 | 安全拒绝 |
| S5_steep_wet | 通过场景 | 否 | 否 |
| S5_steep_wet | 耗时 (s) | 25.0 | 25.0 |
| S5_steep_wet | 平均滑移率 | 1.9% | 0.0% |
| S5_steep_wet | P95 滑移率 | 16.1% | 0.0% |
| S5_steep_wet | 严重打滑占比 | 2.1% | 0.0% |
| S5_steep_wet | 最大侧倾角 (°) | 5.0 | 5.0 |
| S5_steep_wet | 倾翻 | 否 | 否 |
| S5_steep_wet | 后溜距离 (m) | 1.52 | 0.19 |
| S5_steep_wet | 驻车溜坡 (m) | 0.00 | 0.00 |
| S5_steep_wet | 附着估计 MAE | 0.084 | 0.146 |
| S5_steep_wet | 能耗 (Wh/m) | 0.491 | 0.508 |
| S5_steep_wet | 安全介入 | 0 | 0 |
