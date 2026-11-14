# 纯算法 Python SDK 交付与使用

独立分发包为 `route-plan-sdk 0.2.15`，导入名为 `route_plan_sdk`。`plan()` 输入静态场景、车辆位姿和目标，返回有序位姿与拾取安排。`CoveragePolicy` 根据停稳观测和实测反馈逐步输出远近拾取动作。

包内只包含算法和规划必需的私有几何校验内核。不包含 IR-SIM、绘图、动画、交互窗口、演示入口或实车控制功能。该内核与仓库内的几何规则同源，在打包时复制到私有命名空间，避免另行维护一套碰撞实现。

## 分发文件

固定打包入口：`python planner/tools/package_sdk.py`。生成位置：`sim/output/sdk/dist/`。

| 文件 | 用法 |
| --- | --- |
| `route_plan_sdk-0.2.15-py3-none-any.whl` | 在其他工程的 Python 环境中直接安装 |
| `route_plan_sdk-0.2.15.tar.gz` | 标准源码分发包，可直接 pip 安装 |
| `route_plan_sdk-0.2.15-source.zip` | 解压后可查看、修改源码，含完整说明与调用示例 |
| `SHA256SUMS` | 核对所交付文件的 SHA-256 |

无需安装原仓库的 `sim/` 或 `planner/` 包，也无需配置 `PYTHONPATH`。源代码包中包含默认车辆 YAML，默认尺寸直接来自构建时的 `sim/config/robot.yaml`，不在 API 中重复硬编码。

## 依赖与安装

SDK 运行要求 Python `>=3.10`；运行依赖仅为 `PyYAML>=6.0.3`、`ompl==2.0.1`。源码构建使用 Hatchling，由 pip 的隔离构建流程自动安装；Hatchling 不是目标环境的运行依赖。仓库内的打包脚本使用 Python 3.11 以上，业务工程只安装、调用 SDK 时不受此限制。

SDK 不引入 IR-SIM、Matplotlib、NumPy、SciPy、Shapely、Pillow 或 ROS。OMPL 含本地二进制组件，目标平台需要能够安装对应的 OMPL 2.0.1 发行包或提供可用构建。独立环境验证平台为 macOS arm64。v0.1.1 的 Python 3.10.20 wheel 和源码包各通过 14 项公共 API 测试；此前 v0.1.0 另已验证 Python 3.11、依赖隔离和实际 OMPL 绕行。

v0.1.1 热修复通过 69 项仿真测试和 44 项规划器测试。安装后的 Python 3.10 SDK 另回归十组随机场景（种子 41、57、28、38、49、70、1、97、85、54），每组 40 个原目标加 10 个重叠观测，全部 50/50 清除；按 0.013 秒和 0.7 秒两种仿真步长回放，碰撞、剩余目标及 ID 丢失均为零。

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install /完整路径/route_plan_sdk-0.2.15-py3-none-any.whl

# 或在源码解压目录执行
python -m pip install .

# 需要在其他工程中修改本地源码时
python -m pip install -e /完整路径/route-plan-sdk-0.2.15
```

Windows PowerShell 的环境激活命令为 `.venv\Scripts\Activate.ps1`，安装命令相同。具体 OMPL 平台可用性以目标环境实际安装为准。

## 调用方式

```python
from route_plan_sdk import PlanningRequest, Target, plan

request = PlanningRequest(
    request_id="task-001",
    frame_id="map",
    width=12.0,
    height=9.0,
    pose=(3.0, 4.5, 0.0),
    targets=[Target("item-a", 5.0, 4.5), Target("item-b", 6.5, 5.0)],
    target_diameter=0.10,
    method="stops",
    timeout=3.0,
)

response = plan(request)
if response.success:
    for step in response.steps:
        print(step.goal, step.expected_collected_ids)
else:
    print(response.status, response.reason)
```

同样支持 `plan(request_dict)`。JSON 输入示例在 [`request.json`](../planner/sdk/examples/request.json)，完整 Python 示例在 [`quick_start.py`](../planner/sdk/examples/quick_start.py)，文件式调用示例在 [`from_json.py`](../planner/sdk/examples/from_json.py)。分发包内均放在 `examples/` 下。

安装后，这些脚本可以复制到任意业务工程运行；它们没有修改 `sys.path`、引用原仓库或加载显示程序。

## 输入输出协议

远近观测、两级方位缓存与 Gateway 异步反馈使用 `CoveragePolicy.next_action()` 和 `submit_result()`。接入契约见 [SDK 的 Gateway 远近拾取策略](../planner/sdk/README.md#gateway-远近拾取策略) 和 [最小异步示例](../planner/sdk/examples/gateway_coverage.py)。

`PlanningRequest` 接受当前车身中心位姿、目标列表、场地边界、统一目标直径、方法与时限。矩形由 `width / height` 和可选的 `map_offset: [x, y]` 定义，默认左下角为 `(0, 0)`。可选的 `fence` 接收 map 下按顺时针或逆时针排列的有效围栏顶点 `[[x, y], ...]`，单位为米，支持凹多边形且自动闭合。传入后可以省略宽高，输出位姿仍在 map 下。车身安全轮廓与消除器不得在任何运动时刻接触围栏，目标圆也必须完整在围栏内。参见 [围栏示例](../planner/sdk/examples/fence.py)。

请求参数 `fence_enabled` 默认是 `true`。设为 `false` 可保留原顶点并关闭围栏。此时可传入矩形宽高，或省略宽高，让 SDK 围绕当前车位和目标生成临时矩形。规划、局部微调和脱困都使用该开关，输入与输出坐标保持在原坐标系。

`mode="auto_fence"` 以本次 `pose.x/y` 为固定圆心，限制车身中心全程离起点的距离。半径由 `auto_fence_radius_m` 指定，默认 1.4 米。场地多边形围栏开启时同时校验两道边界。圆外物品在消除器能够从圆内覆盖时仍可拾取。圆内找不到完整方案时，`plan()` 返回失败和空路线。

坐标单位为米，yaw 为弧度。`frame_id` 只作关联标识，不执行坐标转换。物体 ID 支持非空字符串和整数，并保持原类型；重复 ID 会报错。v0.1.1 起允许圆心重合及目标圆重叠，重叠连通组一次清除，消除器接触任意成员即在同一步返回整组原始 ID。仅相切不分组。碰撞按清除前全部成员圆的并集检查，车身先接触或同时接触仍判碰撞。初始接触也按整组处理，输入顺序不影响分组。

v0.2.4 起，`robot_config.collector.gap` 设置车头到消除区后沿的距离，独立于 `safety.body_margin`，取非负米数。自定义车型省略 `gap` 时使用 `body_margin`，响应车型返回实际间距。车身安全外扩仍检查四周，圆接触安全矩形时碰撞优先。新场景使用规则版本 3，历史归档按记录版本回放。

实车停靠使用 `method="stops"`。在全部清除且安全的基础上优化拾取停靠次数，路程作为结果统计。v0.2.2 起，任何实际平移大于 5 cm、任何实际转向大于 1°。`pickup_stops[].goal_index` 从 0 开始对应 `goals`，到达完整位姿后一起处理其 `target_ids`。目标在停靠前持续参与碰撞检查。起始位姿已覆盖目标时，可以在起点原地拾取。

v0.2.1 收紧停靠覆盖条件：同批每个 ID 的圆心必须位于消除矩形内，圆边缘相切或相交而圆心在外均不算覆盖；不要求整个圆进入。即使目标彼此重叠，也不清除区域外的圆心。碰撞继续使用完整物理半径。候选、初解、显式拾取及回放使用同一圆心条件，旧 v0.2.0 的相切覆盖统计不能作为本版验证。新记录保存 `pickup_rule="center"`，旧归档缺少该字段时保留旧接触规则回放。参见 [v0.2.1 修正版](releases/sdk-v0.2.1.md)。

`PlanningResponse` 返回：

- `status / success`：是否有通过几何校验的完整计划。
- `goals / steps`：完整有序目标位姿，以及每步预计清除的外部物体 ID。
- `initially_collected_ids / expected_collected_ids / remaining_ids`：初态接触、整份计划预测清除与剩余的目标 ID。
- `length / elapsed / final_pose`：路程、SDK 总耗时和预测终态。
- `reason / metrics / optimization`：原因、搜索诊断及优化信息。
- `resolved_request`：解析为实际车型字典的输入快照，保存后不依赖原车辆配置文件路径。

`response.to_dict()` 返回可 JSON 序列化的数据。失败时 `goals / steps` 为空、`length` 为 `None`；非法输入或配置会抛出 `ValueError`，文件错误可能抛出 `OSError`，应与求解未找到解区分。

在 `feasibility/length` 模式中，目标圆若一开始已接触消除区，会按模型计入 `initially_collected_ids`；`stops` 模式则要求一个显式拾取停靠，初始不自动消除。所有拾取字段均为模型预测，不代表实车反馈。结果需逐条执行，并保持原地转向、直线前进和末端转向的规则；SDK 不发送实际控制指令。

完整协议及约束还可查看分发包的 [README 模板](../planner/sdk/README.md)。

## 仓库内构建与维护

公共接口实现在 [`api.py`](../planner/src/route_plan_planner/api.py)。仓库开发环境也可从 `route_plan_planner` 导入同名 API；独立交付统一使用 `route_plan_sdk`。

SDK 构建配置、说明与 examples 放在 `planner/sdk/`。构建器从当前算法和几何源文件生成独立源码项目，排除显示模块，并将几何依赖改为 SDK 私有命名空间。

```bash
# 常规：一条命令生成源码 ZIP、源码分发包、wheel、BUILD_INFO 和校验文件
python planner/tools/package_sdk.py

# 可选：额外验证独立安装与两个 example；仍不运行全量回归
python planner/tools/package_sdk.py --verify

# 自定义输出目录
python planner/tools/package_sdk.py --output /完整路径/sdk-dist
```

打包工具要求 Python `>=3.11`、PyYAML 和 PATH 中可用的 uv，可使用已配置的 Conda `route-plan` 环境。普通打包只检查 Python 语法、wheel 结构、默认配置资源及依赖，不创建安装环境或执行求解器。构建依赖通过 uv 缓存复用。每次先在临时目录构建，成功后更新固定产物，重复运行不需要删除旧目录。

`--verify` 才创建独立环境并安装 SDK，运行 Python 与 JSON example，确认没有仿真显示依赖。完整算法回归在修改算法时单独执行，不与每次打包绑定。`BUILD_INFO.json` 记录是否做过安装验证，不把常规打包冒充已通过全量测试。

若只需要生成可编辑的源码项目，仍可单独调用 `python planner/tools/build_sdk.py --output 新目录`；该底层命令保留“不覆盖既有源码目录”的行为。

交付副本仅存在于被忽略的 `sim/output/` 中，不提交重复源码或构建产物。后续维护应修改仓库内的原算法、几何或 API 文件，再重新构建 SDK。
