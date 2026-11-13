# Route Plan 纯算法 SDK

`route-plan-sdk 0.2.13`，Python 导入名为 `route_plan_sdk`。输入场地、车辆位姿和圆形目标列表，输出经过几何验证的有序 map 目标位姿。新增 `plan_observation_step()`，用于先观测、再逐站拾取。

包内包含任务规划、路径长度优化与必要的私有几何校验内核。没有 IR-SIM、窗口、绘图、动画、场景演示、ROS 或实车控制功能。无需原仓库目录或 `PYTHONPATH`，默认车型文件已随包携带。

## 依赖

| 依赖 | 版本要求 | 用途 |
| --- | --- | --- |
| Python | `>=3.10` | SDK 运行环境；仓库打包工具要求 3.11 以上 |
| PyYAML | `>=6.0.3` | 默认或自定义车辆配置读取、内部几何输入构建 |
| OMPL | `==2.0.1` | 简单连接受阻时的 RRT 局部路径规划 |
| Hatchling | 构建依赖，由 pip 自动隔离安装 | 从源码安装时构建 wheel；预编译 SDK wheel 安装不需要它 |

不依赖 `route-plan-sim` 分发包、IR-SIM、Matplotlib、NumPy、SciPy、Shapely、Pillow、ROS 或 Conda。只安装本 SDK 会由 pip 解析上表两项运行依赖及其依赖链，不会引入显示功能。

OMPL 含本地二进制组件，目标平台需要能够安装 `ompl==2.0.1` 的对应发行包，或自行提供可用构建；本 SDK 自身代码是纯 Python。`use_ompl=False` 可在调用时关闭 RRT 回退，但标准安装仍声明 OMPL 依赖，以保留已验证的算法能力。

建议使用独立环境：

```bash
python3.10 -m venv .venv
# macOS / Linux
source .venv/bin/activate
# Windows PowerShell 使用 .venv\Scripts\Activate.ps1

# 从解压后的源码项目目录安装
python -m pip install .

# 或安装提供的 SDK wheel，二者任选一个
python -m pip install /完整路径/route_plan_sdk-0.2.13-py3-none-any.whl
```

源码开发可使用 `python -m pip install -e /完整路径/route-plan-sdk-0.2.13`。不需要同时安装原仓库的 `sim/` 或 `planner/` 工程。

## 最小 Python 调用

```python
from route_plan_sdk import PlanningRequest, Target, plan

request = PlanningRequest(
    width=12.0,
    height=9.0,
    pose=(3.0, 4.5, 0.0),  # 米、米、弧度；车身中心
    targets=[
        Target("item-a", 5.0, 4.5),
        Target("item-b", 6.5, 5.0),
    ],
    target_diameter=0.10,
    method="stops",
    timeout=3.0,
)

result = plan(request)
if result.success:
    for step in result.steps:
        print(step.goal, step.expected_collected_ids)
else:
    print(result.status, result.reason)
```

完整例子：

```bash
python examples/quick_start.py
python examples/from_json.py examples/request.json --output plan.json
python examples/multi_pickup.py
```

`quick_start.py` 可以复制到其他工程中运行，只有 SDK 导入，没有修改 `sys.path` 或引用原仓库。JSON 调用直接 `plan(request_dict)` 即可；输入示例见 `examples/request.json`。

## 分段观测与拾取

每次停稳后，把实测车身中心位姿和最近确认的未拾取目标放入 `PlanningRequest` 的 `pose` 和 `targets`。把剩余粗测目标另传给 `plan_observation_step(request, coarse_targets, ...)`。粗测条目含 `id, x, y, error_radius`，其中误差半径是圆心位置的最大误差，单位为米，目标本身的物理半径由 SDK 另算。各坐标统一使用 `map`，同一 ID 的精确坐标优先。SDK 按输入检查碰撞，调用端须提供会影响运动的目标或保守占用信息。地图尺寸和可选 fence 沿用 `PlanningRequest`。

默认可靠距离为车身中心起算的 1.4 m，水平视角为车头两侧各 30°。可通过 `reliable_range` 和 `horizontal_fov_deg` 配置，两者用于挑选观测位。哪些目标实际测准，由调用端根据新观测决定。SDK 使用静态目标快照，调用端保留未拾取目标，处理关联、遮挡和拾取确认。

| `status` | 执行方式 |
| --- | --- |
| `observe` | 只执行本次 `goals`。停稳后获取新帧，再用实测位姿调用。 |
| `pickup` | 执行 `goals` 至 `pickup_stops` 标记的拾取位，按 ID 拾取。收到实际成功反馈后才删除该 ID。 |
| `blocked` | 保持停车。记录 `reason` 和 `rejections`，获得新信息后再调用。 |
| `no_targets` | 当前请求没有目标。 |

`observation_pose` 是计划观测位，可能需要多次平移到达。`pending_hops` 是供下次调用重新验证的剩余路段，执行端只下发本次 `goals`。每次把近期停过的位姿放入 `visited`，并回传 `observations_since_pickup`。如果新测量确认了目标、目标位置更新或执行偏离计划，清空 `pending_hops` 再调用。每段依次执行原地转向和向前直行，碰撞检查覆盖整段。平移和转向仍须超过 5 cm 和 1° 的运动死区。

`initial_clearance_confirmed` 默认 `False`。只有本次起始位姿的完整安全外扩区已独立确认无目标接触，才传 `True`。这种情况下，如果粗测误差圈覆盖了起始车身，SDK 可以在确认新扫过的区域无碰撞风险后允许直行；转向仍检查完整扫掠。输入漏检或真实误差超出 `error_radius` 会使模型低估风险。

运行 `python examples/observation.py` 查看停稳重测后转入拾取的最小例子。`result.to_dict()` 可直接保存为 JSON。

### 实车回放日志

建议每趟保留一份按时间排列的 JSONL。观测、规划、执行与拾取用同一个 `run_id` 关联，每次调用带 `request_id`。使用统一时钟，或同时记录各消息的原始时间戳与对应时钟域。

| 什么时候记录 | 需要的字段 |
| --- | --- |
| 每趟开始 | SDK 版本、车辆配置、`fence_enabled`、实际使用的 fence 顶点、坐标系、相机/TF 标定版本、可靠距离和视场配置 |
| 取帧时 | 帧 ID、采集时间、相机或车体系下原始检测、当时的 TF 与转换后的 `map` 坐标、目标 tracking ID、远近/确认标记、误差半径。分别标记空帧和取帧失败 |
| 每次规划 | 函数名、完整输入和 `to_dict()` 输出。观测规划保留 `visited`、`pending_hops`、`initial_clearance_confirmed` 及其依据。异常保留错误类型和信息 |
| 每段运动 | 下发的目标位姿及索引、带时间戳的实际位姿序列、停稳时的实测位姿、取消/超时/急停/碰撞反馈 |
| 每次拾取 | 停靠索引、预期 ID、逐 ID 的实际结果、拾取前后复测帧 ID、失败或物体掉落的位置 |

同一目标的远距粗测与靠近后的精测尽量保留同一 tracking ID。成对记录能用来拟合误差随距离和方向的变化；实际轨迹和停稳帧能定位误差出在观测、执行还是拾取。优先保留卡住、误停、擦碰和漏拾的完整轮次，另附几条正常完成轮次作为对照。原始图像和 ROS bag 可以仅为关键轮次保留。

## 独立脱困

`plan_escape(request)` 接收同一份地图、当前车位、车辆配置和全部观测目标。它搜索直行、原地转向或一次转向后直行，直到所有目标都离开车身外扩区。对起始就进入外扩区的物体，实体车身必须始终有净空，且距离全程不能减小。其他物体与地图边界按常规规则检查。

`EscapeResponse.goals` 是按顺序执行的位姿，`initial_intrusion_ids` 标识起始侵入物，`final_pose` 是预计脱离后的位姿。脱困期间保留全部目标。状态为 `already_clear` 时可以进入常规规划。状态为 `body_contact` 或 `blocked` 时返回空动作。执行成功后，用实测位姿和最新目标重新调用 `plan()`。

## 到站后的局部拾取修正

```python
from route_plan_sdk import plan_pickup_adjustment

# updated_request 的 pose 是停车后的实测位姿，targets 是最新观测的未拾取目标。
result = plan_pickup_adjustment(updated_request, target_ids=["item-a"], max_displacement=0.40)
if result.success:
    print(result.goals, result.pickup_stops, result.remaining_ids)
else:
    print(result.status, result.reason)
```

实测位姿与目标圆心使用同一个固定坐标系。无图时，用字典传入请求并省略 `width` 和 `height`。可以把停车时刻的车身坐标系固定下来，传 `pose=[0, 0, 0]`，目标坐标和返回的全部位姿都相对这个起点。有已知矩形边界时，传入宽高即可检查越界。

把附近其他未拾取目标也放在 `targets` 里，它们会参与全程碰撞检查。`max_displacement` 是车身中心距起点的最大距离，单位米，默认 0.40。平移和转向沿用停靠模式的死区。

成功时返回一个拾取停靠，指定的全部 ID 都在该位姿的消除矩形内。如果它们已经在区内，可以直接原地拾取。`remaining_ids` 保留该停靠未清除的目标。拾取确认后，剩余任务用实测位姿和最新目标重新规划。

起点有物体侵入安全外扩区时，接口先尝试脱困，再规划拾取。脱困时保持实体车身与侵入物的净空，已退出外扩区的物体恢复常规碰撞检查。`metrics.escape_goal_count` 标出 `goals` 开头有几条脱困动作。脱困和对位共用局部位移上限。物体触碰实体车身时返回 `body_contact`，找不到安全的完整路线时返回空动作。

## 输入约定

- `fence_enabled`：布尔值，默认 `true`。设为 `false` 时可以保留围栏顶点，规划使用显式传入的矩形宽高。宽高都省略时，SDK 根据车位和目标生成临时矩形，输入与输出保留原 map 坐标。拾取规划、局部修正和脱困都读取此开关。
- `width / height`：矩形场地宽高，单位米。矩形左下角通过可选的 `map_offset: [x, y]` 指定，默认 `[0, 0]`。`plan()` 和 `plan_escape()` 在未提供围栏且未显式关闭围栏时要求传入宽高。局部修正支持省略两项。
- `fence`：可选的有效场地边界，格式为 map 米制坐标 `[[x1, y1], [x2, y2], ...]`，顺时针或逆时针排列，自动闭合。支持凹多边形、斜边和负坐标。边界为单个无孔、无自交多边形。启用时以围栏为准，省略宽高时由围栏外接矩形计算。显式宽高与原点可用于关闭围栏后的矩形模式。已经内缩的实测顶点可直接传入。
- `pose`：当前车辆中心 `(x,y,yaw)`。所有目标与位姿使用同一坐标系；yaw 为弧度，逆时针为正，0 朝右。
- `targets`：`Target(id,x,y)` 列表，坐标为圆心。ID 可为非空字符串或整数，输出保留原 ID 类型；重复 ID 会报错。允许圆心重合及圆形目标重叠，所有原始 ID 都保留。
- `target_diameter`：全部物体统一直径，默认 0.10 米；显式传 0 可表示历史点模型。
- `method`：`feasibility`、`length` 或 `stops`，默认 `length`。实车停靠模式显式传 `method="stops"`。在全部清除且无碰撞的前提下，此模式优化拾取停靠次数，路程作为结果统计。每段实际平移大于 5 cm，每次实际转向大于 1°，包含平移前对准和到位后的末端转向。预算耗尽时返回已验证的可行路线（如果已找到）。运动死区是内置规则，无需增加输入字段。
- `timeout`：搜索预算，默认 3 秒；包括取得可行初解的耗时，最终回放和响应整理可能增加调用总耗时。
- `robot_config`：可选车辆 YAML 路径或完整配置字典，省略时使用随包默认车型。默认车身长宽高为 0.702/0.610/0.169 米，高度仅作记录。消除区左右宽 0.190 米、前后长 0.090 米，每侧安全外扩 0.050 米。`collector.gap` 设置车头到消除区近边的距离，默认 0.183 米。自定义车型省略 `gap` 时使用 `body_margin`。响应的 `resolved_request.robot_config` 返回实际车型。
- `request_id / frame_id`：可选关联标识，随响应返回；`frame_id` 默认 `map`，仅作标识，不执行坐标变换。
- `use_ompl / seed`：是否使用 RRT 回退及随机种子。OMPL 在每个进程第一次使用时初始化随机流，同进程后续调用沿用该流，不承诺重新搜索逐位复现。

输入目标圆必须在场地内，并与实体车身有净空。`stops` 模式起点有目标侵入安全外扩区时，先尝试脱困，再规划全程拾取。`metrics.escape_goal_count` 标出 `goals` 开头必须按序执行的脱困动作数量。旧 `feasibility/length` 模式要求起点目标与安全轮廓分离，按运动中的接触消除，初始接触消除区的目标记录在 `initially_collected_ids`。

`examples/fence.py` 演示围栏调用。车身安全外扩和消除器在直行、转向全程都必须严格位于围栏内，目标圆也不能碰边。围栏保存在 `resolved_request.fence`，可随规划结果一起归档回放。使用围栏时，独立脱困和局部修正可以接收同一份围栏。独立固定障碍物和栅格地图仍不属于 SDK 请求字段。

`stops` 模式只在指定拾取停靠位消除目标，沿途和转向时不自动删除。即使目标一开始就在消除区，也需要一个显式拾取停靠。v0.2.1 起，合并一批的每个目标圆心都必须位于同一位置、同一朝向下的消除矩形内（含圆心在边界上，仅容许数值误差）；圆边缘相切或部分重叠、圆心在外均不算覆盖。不要求整个圆都进入。碰撞始终按完整圆计算并优先处理。目标在该停靠之前一直作为障碍；途中先后经过多个目标不会被算作“一停全清”。

旧接触模式仍按重叠连通组清除，保留原始 ID。`stops` 模式逐 ID 检查圆心；彼此重叠的多个目标可以同时拾取，但区域外的圆心不能因为重叠关系被顺带清除，必要时分批处理。所有未清除圆都参与碰撞检查，不能用质心替代整组碰撞范围。

车身四周（包括正前方）的 `body_margin` 区域统一参与连续碰撞检查，圆与安全矩形接触即失败，不能因位于消除器方向而豁免；转动是否可行也由完整扫掠过程判断。不另增加“只能前进拾取”的独立规则。

## 输出约定

`PlanningResponse` 的主要字段：

| 字段 | 含义 |
| --- | --- |
| `success / status` | 是否找到通过完整校验的计划；`success` 为布尔属性 |
| `goals` | 有序 `(x,y,yaw)` 位姿，可含纯转向和中间位姿 |
| `steps` | 与 goals 一一对应的位姿及该步预计清除的外部目标 ID |
| `pickup_stops` | `stops` 模式的拾取停靠列表：`goal_index`、`pose`、`target_ids`；索引从 0 开始，对应 `goals` |
| `stop_count` | 拾取服务位姿数量；仅 `stops` 模式成功时为整数，旧模式为 `None`，不能用 `len(goals)` 代替 |
| `expected_collected_ids / remaining_ids` | 模型预计清除的目标和剩余目标 ID |
| `initially_collected_ids` | 在初始消除区已发生接触的目标 ID |
| `length` | 成功计划的总平移路程，米；无完整计划时为 `None` |
| `elapsed` | SDK 调用耗时，秒，包含输入构建与验证，不是车辆执行时间 |
| `final_pose` | 模型预计的车辆终态，米、米、弧度 |
| `reason / metrics / optimization` | 失败原因、搜索计数、优化效果 |
| `resolved_request` | 使用实际解析后的车型字典保存的输入快照，可独立序列化、重新调用 |
| `request_id / frame_id / schema_version` | 关联标识与协议版本，当前版本为 1 |

`result.to_dict()` 返回 JSON 原生容器，可使用 `json.dumps(..., allow_nan=False)` 保存。规划失败时不返回可执行的 `goals/steps`；未找到解不表示已证明场景无解。非法输入或车辆配置会抛出 `ValueError`，文件问题可能抛出 `OSError`，集成时应与求解失败区分处理。

## 执行与集成边界

计划只能按顺序执行：每一位姿按“原地转向目标位置 → 直线前进 → 原地调整最终朝向”完成，上一条完成后再发下一条。同位置的连续 yaw 目标可能编码特定转向方向，不要自行合并。

在 `stops` 模式中，到达 `pickup_stops[].goal_index` 指定的完整位姿后，一次处理其 `target_ids`。实际拾取确认完成后再继续后面的位姿，不能只取目标顺序再重算接近路径。失败时调用工程应使用实际位姿和剩余目标重新规划。`stop_count` 统计拾取服务位姿，不统计中间转向和避让带来的控制停车；同位置不同朝向的两次拾取也属于两个服务位姿。

起始位姿已经覆盖目标时，首条 `goal` 可以与起始位姿相同，并在 `pickup_stops` 中标记。执行端可在原地直接触发这次拾取。

`python examples/multi_pickup.py` 演示两个非重叠目标在一个服务位姿中一次清除。

`success` 与 `expected_collected_ids` 只表示该模型中的安全、清除预测，不是实车执行或拾取确认。SDK 不输出轮速、速度指令、消除器开关或机械动作，不模拟定位误差与拾取失败。

规划期间场景应保持静态。每次 `plan()` 都创建独立输入快照和规划器，不修改请求对象；同进程建议串行调用，若需取消或隔离任务，可由外部进程管理。返回结果之后的实际执行、反馈与重规划由调用工程管理。
