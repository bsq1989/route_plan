# Gateway 实车复盘日志清单

适用于 `route-plan-sdk 0.2.15`（`f6416be`）的 `CoveragePolicy` 远近拾取流程。每趟从创建策略实例开始记录，直到任务完成、停止或取消。用这份日志还原算法看到的观测、生成的动作、底盘实走的路线和每件物品的拾取结果。

## 每趟需要的日志

沿用 Gateway 的任务 ZIP 和 JSONL 格式。已有 `manifest.json`、`gateway-events.jsonl` 和 SDK 输入输出文件可以继续复用。下列是建议新增或补齐的内容。

| 何时记录 | 保留内容 |
| --- | --- |
| 任务开始 | `run_id`、`task_id`、策略实例标识、SDK 版本和构建提交（可直接附 `BUILD_INFO.json`）、Gateway 版本、实际生效的完整 `CoverageConfig`、`use_deferred_bearings`、`request_options`。策略重建时另记一次初始化，保留触发原因。 |
| 配置快照 | 实际使用的车辆与消除器几何、安全外扩、目标直径、`fence_enabled` 和生效的围栏顶点或矩形边界。保留 `precision_range_m`、`auto_fence_radius_m`、视距、视角和 `position_match_tolerance_m` 的实际值。附上相机标定版本与转换关系，包括 `base_link` 到模型车身中心、实际旋转中心及相机的关系。外部配置文件随包附原文。 |
| SDK 给出动作 | 记录每个不同 `action_id` 对应的完整 `CoverageAction`，包括 `kind`、`frame_id`、`start_pose`、`goals`、`pickup_stops`、`targets`、`distance_m`、`auto_fence_radius_m`。附生成时间及关联观测帧。 |
| Gateway 回传结果 | 保存传给 `submit_result()` 的原始参数，含 `action_id`、`status`、停稳后实测 `pose`、`time_s`、`reason`、`picked_ids` 和适用时的整个 `observation`。记录调用返回的 `True` / `False`，或异常类型和信息。迟到、重复、受阻及取消回执也保留。 |
| 局部 `plan()` 求解 | 记录每次实际送入的完整请求和 `response.to_dict()`，含 `resolved_request`、状态、原因和耗时。用规划序号和观测帧关联。如果生成了动作，再关联 `action_id`。求解失败可能没有对应的拾取动作，也要保留输入及返回或异常。 |
| 任务结束 | Gateway 最终状态、策略 `stop_reason`、触发结束的动作或观测、确认已拾与仍待确认的目标。实测剩余物品的数量或位置如果已核对，也一起记录。 |

每条事件保留原有 `seq` 和时间戳，用 `run_id + task_id` 串起整趟，用 `action_id` 串起 SDK 动作和回执。局部拾取 ID 只在该动作内有效，用 `action_id + target_id` 查对应的 `action.targets`。动作被 Gateway 拆成多个 ROS goal 时，记录 `goal_index` 与实际 ROS goal UUID 的映射。局部规划日志可通过 `CoveragePolicy(planner=logged_plan)` 接入，在包装现有 `plan()` 时记录请求、返回或异常。

## 观测与实车执行细节

- **每次 `observe`。** 原样保存 SDK 收到的 `observation_id`、`frame_id`、`detections` 及其原有顺序。精测保留 `quality="precise"` 和 `xy_m`。方位保留 `quality="bearing"`、`origin_xy_m` 和 `bearing_map_rad`。同一帧可同时包含两类检测。有效空帧保留 `detections: []`，取帧失败记录状态及原因。
- **坐标和时钟。** SDK 日志保留实际传入的全精度数值，坐标用米，yaw 和方位角用弧度，默认在 `map`。保留相机原始采集时间、对应的 TF 时间与实际变换、停稳时间和回执时间。若给 SDK 传了 `observation.time_s`，与回执 `time_s` 使用同一时钟。不同设备时钟需保留时钟域和对齐信息。原始相机检测和可用的 `track_id` 作为诊断线索，SDK 复跑使用实际收到的帧载荷。
- **每段转向或前进。** 除计划 `goals`，保留 Gateway 真正下发的路径或位姿、使用的 TF、起止实测位姿、带时间戳的实际轨迹，以及到达、受阻、超时、取消或急停原因。若安全检查拒绝动作，尽量留拒绝检查项、首次触发的位置和所依据的占用或围栏数据。远处 `turn` / `forward` 的安全检查在 Gateway。
- **每次实际拾取。** 留停靠索引、预期 ID 和对应坐标、拾取前实测位姿、拾取前后观测帧、逐 ID 结果及失败原因。部分成功时保留已成功、失败和未执行的目标。传入 SDK 的 `picked_ids` 仅含已确认成功的 ID。

## 二级 hint 复盘

联调时如能附策略诊断，保存每次观测后的 `latest_frame.known` 和 `latest_frame.deferred`，以及本趟的 `decisions`、`approaches`。同时保留诊断中用的 SDK `frame_index` 与 Gateway `observation_id` 的映射。

查漏拾或折返时，优先对照候选 yaw、分数、选中方向和受阻原因。查二级线索时，在 `approaches` 找 `source="deferred_bearing"`，核对旧射线原点和方向、复测帧及 `confirmed`。二级缓存会随观测时间和执行结果更新，因此复盘需要从任务起点顺序喂入回执。

原始图像和 ROS bag 可优先保留碰撞、急停、长时间原地转、折返和漏拾的轮次，再附一趟正常完成的对照。录包要覆盖出问题的动作前一帧和结束后的实测反馈。碰撞或坐标问题优先留 TF、定位与实际轨迹、速度或控制指令、安全输入及相机检测。若录包由独立服务保存，在任务 ZIP 里留 bag 文件名或可获取路径、时段和对应的动作 ID。

## 导出后怎么检查

1. 从任务初始化开始，按 `seq` 给同版本的 `CoveragePolicy` 回灌记录。每一步先对比新策略动作的类型、目标位姿和拾取停靠，再提交原始实测回执。
2. 新实例会生成新的 `action_id`。按动作输出顺序建立旧 ID 到新 ID 的映射，回放回执时使用对应动作的新 ID。迟到或重复回执仍关联原动作，核对是否被忽略。
3. 如果求解预算或 OMPL 让局部路线出现差异，用记录的 `plan()` 输出回放上层决策，再用同一请求单独复算局部规划。优先定位第一个出现差异的动作。

建议先交一趟完整任务 ZIP，能从首次 `observe` 走到最终 `stop_reason`，并能从一条拾取回执查到实际 ROS goal 和目标坐标，就能开始复盘。 [SDK 接入约定](../planner/sdk/README.md#gateway-远近拾取策略)
