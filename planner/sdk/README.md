# Route Plan 纯算法 SDK

`route-plan-sdk 0.1.0`，Python 导入名为 `route_plan_sdk`。输入静态场地、车辆位姿和圆形目标列表，输出经过几何验证的有序 map 目标位姿。

包内包含任务规划、路径长度优化与必要的私有几何校验内核。没有 IR-SIM、窗口、绘图、动画、场景演示、ROS 或实车控制功能。无需原仓库目录或 `PYTHONPATH`，默认车型文件已随包携带。

## 依赖

| 依赖 | 版本要求 | 用途 |
| --- | --- | --- |
| Python | `>=3.11` | 运行环境 |
| PyYAML | `>=6.0.3` | 默认或自定义车辆配置读取、内部几何输入构建 |
| OMPL | `==2.0.1` | 简单连接受阻时的 RRT 局部路径规划 |
| Hatchling | 构建依赖，由 pip 自动隔离安装 | 从源码安装时构建 wheel；预编译 SDK wheel 安装不需要它 |

不依赖 `route-plan-sim` 分发包、IR-SIM、Matplotlib、NumPy、SciPy、Shapely、Pillow、ROS 或 Conda。只安装本 SDK 会由 pip 解析上表两项运行依赖及其依赖链，不会引入显示功能。

OMPL 含本地二进制组件，目标平台需要能够安装 `ompl==2.0.1` 的对应发行包，或自行提供可用构建；本 SDK 自身代码是纯 Python。`use_ompl=False` 可在调用时关闭 RRT 回退，但标准安装仍声明 OMPL 依赖，以保留已验证的算法能力。

建议使用独立环境：

```bash
python3.11 -m venv .venv
# macOS / Linux
source .venv/bin/activate
# Windows PowerShell 使用 .venv\Scripts\Activate.ps1

# 从解压后的源码项目目录安装
python -m pip install .

# 或安装提供的 SDK wheel，二者任选一个
python -m pip install /完整路径/route_plan_sdk-0.1.0-py3-none-any.whl
```

源码开发可使用 `python -m pip install -e /完整路径/route-plan-sdk-0.1.0`。不需要同时安装原仓库的 `sim/` 或 `planner/` 工程。

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
    method="length",
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
```

`quick_start.py` 可以复制到其他工程中运行，只有 SDK 导入，没有修改 `sys.path` 或引用原仓库。JSON 调用直接 `plan(request_dict)` 即可；输入示例见 `examples/request.json`。

## 输入约定

- `width / height`：矩形场地宽高，单位米，内部原点 `(0,0)`。独立固定障碍物、栅格地图和多边形边界尚未支持。
- `pose`：当前车辆中心 `(x,y,yaw)`。所有目标与位姿使用同一坐标系；yaw 为弧度，逆时针为正，0 朝右。
- `targets`：`Target(id,x,y)` 列表，坐标为圆心。ID 可为非空字符串或整数，输出保留原 ID 类型；重复 ID、重复或重叠目标会报错，不会静默丢点。
- `target_diameter`：全部物体统一直径，默认 0.10 米；显式传 0 可表示历史点模型。
- `method`：`feasibility` 或 `length`，默认 `length`。先求完整安全路线，长度优化不能改善时保留原路线，不承诺全局最优。
- `timeout`：搜索预算，默认 3 秒；包括取得可行初解的耗时，最终回放和响应整理可能增加调用总耗时。
- `robot_config`：可选车辆 YAML 路径或完整配置字典，省略时使用随包默认车型。默认车身长宽 0.50/0.60 米，消除区宽深 0.35/0.24 米，每侧安全外扩 0.10 米。参考点及旋转中心暂按车身中心，消除器暂按居中紧贴车头。
- `request_id / frame_id`：可选关联标识，随响应返回；`frame_id` 默认 `map`，仅作标识，不执行坐标变换。
- `use_ompl / seed`：是否使用 RRT 回退及随机种子。OMPL 在每个进程第一次使用时初始化随机流，同进程后续调用沿用该流，不承诺重新搜索逐位复现。

输入目标圆必须在场地内，不能触碰当前车身安全轮廓。目标与消除区接触时，模型会立即视为已清除，并在 `initially_collected_ids` 中标记。该结果仍是模型预测。

## 输出约定

`PlanningResponse` 的主要字段：

| 字段 | 含义 |
| --- | --- |
| `success / status` | 是否找到通过完整校验的计划；`success` 为布尔属性 |
| `goals` | 有序 `(x,y,yaw)` 位姿，可含纯转向和中间位姿 |
| `steps` | 与 goals 一一对应的位姿及该步预计清除的外部目标 ID |
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

`success` 与 `expected_collected_ids` 只表示该模型中的安全、清除预测，不是实车执行或拾取确认。SDK 不输出轮速、速度指令、消除器开关或机械动作，不模拟定位误差与拾取失败。

规划期间场景应保持静态。每次 `plan()` 都创建独立输入快照和规划器，不修改请求对象；同进程建议串行调用，若需取消或隔离任务，可由外部进程管理。返回结果之后的实际执行、反馈与重规划由调用工程管理。
