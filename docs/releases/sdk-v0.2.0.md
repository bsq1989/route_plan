# SDK v0.2.0 多目标停靠优化版

发版日期：2026-11-08。Git 标签：`sdk-v0.2.0`。纯算法 SDK 为 `route-plan-sdk 0.2.0`，仓库规划器为 `route-plan-planner 0.3.0`；Python SDK 继续支持 Python 3.10 以上，依赖不变。

新增 `method="stops"`，在全部清除且满足碰撞、运动规则的基础上，按字典序优化 `(拾取服务位姿数量, 总平移路程)`。旧 `length` 默认值及 `feasibility` 模式保留。停车优先可能增加路程，不要求全局最优。

## 同一停靠位处理多个目标

候选位姿由单目标、近邻目标对的中点及多个朝向生成。每个候选都计算该位姿下消除区实际覆盖的目标集合和车身安全矩形会触碰的目标集合。分组依据是具体位置、朝向下的几何覆盖，不引入需要调参的空间聚类半径；原有重叠别名组继续保留全部 ID。

采用有限宽度搜索，优先覆盖更多剩余目标，并保留少停靠、短路程的分支。运动连接器逐段检查全部剩余目标和地图边界。搜索预算耗尽时，保留已验证的可行路线；每个最终结果再次独立逐帧回放。

初解由已有接触模式的安全计划产生，再在解析接触时刻精确拆成服务位姿。沿途不同时间碰到的物体分别计数，不依赖显示帧采样，也不直接用 `len(goals)` 计数。

`stops` 模式使用 `collection_mode="at_stop"`。途中和原地转向不自动消除目标；到达指定完整位姿后才消除该位姿实际覆盖的目标。未拾取的目标始终参与连续碰撞检查，正面外扩区域也没有豁免。初始位姿已覆盖的目标仍计作一次显式拾取服务。

## 集成接口

```python
response = plan({**request, "method": "stops"})
print(response.stop_count, response.length)
stops_by_index = {stop.goal_index: stop for stop in response.pickup_stops}
for index, goal in enumerate(response.goals):
    print("依次执行完整目标位姿", goal)
    if index in stops_by_index:
        print("到位后一起拾取并确认", stops_by_index[index].target_ids)
```

示例只展示执行协议，不发送实车指令。`goal_index` 从 0 开始，`pose` 与对应 `goals` 一致。`steps[].expected_collected_ids` 只在拾取停靠处非空。目标 ID 类型保留；旧方法的 `stop_count` 为 `None`。

这里计数的是拾取服务位姿，中间避让/转向带来的物理停车不在该指标内。同位置不同朝向的两次拾取计作两个服务位姿。实车需逐条执行位姿并在停靠处完成拾取确认；仿真成功不代表机械拾取已经成功，失败后由集成方以最新状态重新规划。

## 仿真验证

20 个场景，每组 40 个目标：10 组稀疏随机分布、10 组目标簇（10 簇，每簇 4 个互不重叠的圆）。全部使用默认车型、直径 10 cm、`body_margin=10 cm`，消除区整体前移同样距离。A 为已有长度模式，B 复用同一份 A 初解继续做停靠优化。A 搜索预算 1 秒；B 的总预算 3 秒包含 A 已消耗时间，最终独立回放额外计时。

| 场景 | A 拾取服务数 | B 拾取服务数 | A 路程 | B 路程 |
| --- | ---: | ---: | ---: | ---: |
| 10 组随机 | 400 | 387 | 396.71 m | 446.68 m |
| 10 组目标簇 | 400 | 100 | 197.80 m | 205.67 m |
| 总计 | 800 | 487 | 594.51 m | 652.35 m |

A、B 均为 20/20 全部清除且无碰撞，800 个原始目标 ID 保留，按 0.013 秒、0.7 秒两种步长回放通过。簇场景拾取服务数减少 75%；总体减少 39.1%，总路程增加约 9.7%。这是固定集合的一次验证，效果取决于分布与预算；不声称任意场景都减少停车或缩短路程。

测试：70 项仿真测试、54 项规划器测试通过。新增测试覆盖单停靠同时覆盖三个不重叠目标、沿途多目标不能虚算一次、到位前保持障碍、解析旋转接触拆分、预算耗尽保留初解、SDK 输出和 CLI 独立回放。

独立 Python 3.10.20 环境中，wheel 与源码包各通过 23 项 API/停靠测试（另有 1 项 CLI 测试按 SDK 不携带 CLI 的约定跳过）；安装后的 SDK 又独立回放全部 20 组冻结场景，800 个目标全清、无碰撞。独立 Python 3.11 安装及三个 example 也通过。

摘要：[停靠 A/B 记录](../../planner/benchmarks/stops-ab-v1.json)。复现：

```bash
python planner/tools/benchmark_stops.py --render
uv run --project planner route-plan solve --method stops --scenario random --seed 42 --timeout 3 --output sim/output/stops-plan.json
uv run --project planner route-plan replay sim/output/stops-plan.json
python planner/tools/package_sdk.py --verify
```

构建产物位于 `sim/output/sdk/dist/`，包含源码 ZIP、标准源码分发包、wheel 和校验文件；不携带显示/仿真应用，也不发布到公共包索引。
