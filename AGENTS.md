# 项目与仿真工具说明

## 项目目标与目录

本项目研究前置消除器小车的目标消除顺序与路径规划。已有基于 IR-SIM 的二维几何仿真沙盒和 `planner/` 静态完整准确观测的自动清除 baseline。规划器每次调用处理一份静态目标快照。

- `sim/`：独立仿真工程，拥有自己的 `pyproject.toml` 和 `uv.lock`。仿真代码、配置、依赖和测试均放在此目录。
- `sim/config/robot.yaml`：默认车辆配置，每次新建 `Scene` 自动读取。
- `sim/src/route_plan_sim/world.yaml`：地图、初始位姿和场景随机参数。
- `sim/src/route_plan_sim/scene.py`：几何状态、目标管理、消除与碰撞判定。
- `sim/src/route_plan_sim/drawing.py`：按 map 米制尺寸绘制目标圆，手动窗口与规划器共用。
- `sim/src/route_plan_sim/control.py`：map 目标位姿执行器 `PoseController`。
- `sim/src/route_plan_sim/config.py`：车辆配置加载、校验及场景配置合并。
- `sim/src/route_plan_sim/app.py`：IR-SIM 显示适配、中文界面与手动控制。
- `sim/tests/`：配置、规则、控制器和离屏界面测试。
- `sim/output/`：导出的 PNG、GIF、JSON 和临时构建产物，已被 Git 忽略。
- `planner/`：独立的任务搜索、运动连接器、OMPL 绕行、自动演示及回归测试，包含自己的 `pyproject.toml` 和 `uv.lock`。
- `planner/src/route_plan_planner/api.py`：业务请求与响应封装；`planner/sdk/`：纯算法 SDK 构建配置、依赖说明与 examples；`planner/tools/package_sdk.py`：固定打包入口。
- `planner/src/route_plan_planner/stops.py`：`method="stops"` 停靠优化；`pickup_stops` 给出从 0 开始的目标位姿索引和批次 ID。此模式使用 `collection_mode="at_stop"`，到位后显式 `collect_at_stop()`，途中不删除目标。每段实际平移必须大于 5 cm，每次实际转向必须大于 1°。在运动、碰撞与全部清除约束下仅优化拾取服务位姿数量。旧模式继续按运动接触消除。回放时必须传入 `pickup_stops`，不能用路径经过的多个点冒充一次停靠。
- `docs/static-baseline.md`：静态 baseline 的验证记录与能力边界；`docs/planning-roadmap.md`：分阶段开发计划。
- 任务规划算法通过仿真公开接口调用，不将策略代码混入显示层。根本目标是全部清除且遵守规则；`--method length` 在可行性初解基础上优化总平移路程，不要求全局最优，默认 `--method feasibility` 保留原 baseline。

更完整的交互说明见 `sim/README.md`。

实测围栏静态配置在 `sim/config/measured-fence.yaml`，可给 `route-sim` 和 `route-plan solve` 的 `--config` 使用，场景选 `--scenario random`。`custom.fence_enabled` 开启时使用 `custom.fence` 的 map 米制顶点，关闭时使用 `world.width`、`world.height` 和 `world.offset` 定义的矩形并保留顶点。实测顶点已经内缩 16 cm，直接交给仿真与 SDK。`PlanningRequest.from_scene(scene, method="stops")` 将同一份场地、车位、目标和车辆配置转成 SDK 请求。

## 启动与验证

以下命令均从仓库根目录运行，使用 `uv` 管理 Python 环境和依赖：

```bash
# 打开交互窗口
uv run --project sim route-sim

# 选择预设：front 前方点群、surround 四周包围、random 随机分布
uv run --project sim route-sim --scenario surround
uv run --project sim route-sim --scenario random --seed 123

# 指定其他车辆或地图配置
uv run --project sim route-sim --robot-config /完整路径/robot.yaml
uv run --project sim route-sim --config /完整路径/world.yaml

# 无窗口导出 PNG 和同名 JSON
uv run --project sim route-sim --snapshot sim/output/scene.png

# 测试
uv run --project sim python -m unittest discover -s sim/tests -v

# 静态自动清除及规划器测试
uv run --project planner route-plan solve --scenario random --seed 42 --show
uv run --project planner python -m unittest discover -s planner/tests -v

# 路径长度优化和同场景 A/B
uv run --project planner route-plan solve --method length --scenario random --seed 42 --timeout 3 --show
uv run --project planner route-plan compare --seeds 0:100 --timeout 3 --output sim/output/length-ab
```

也可使用仓库 `environment.yml` 创建独立 Conda `route-plan` 环境，再从根目录执行 `python -m pip install -e ./sim -e ./planner`。具体命令见 `planner/README.md`。

交互模式需要桌面图形环境和中文字体；离屏导出不打开窗口。`--heading`（度）和 `--distance`（米）可在打开窗口或截图前依次执行一次转向和前进。

## 默认车辆与配置约定

以 `sim/config/robot.yaml` 为车辆参数的来源，不在地图文件或算法中重新硬编码默认尺寸：

| 参数 | 当前值 |
| --- | --- |
| 车长、车宽、车高 | 0.702 m、0.610 m、0.169 m |
| 消除区横向宽度、向前深度 | 0.190 m、0.090 m |
| 每侧车身外扩 | 0.050 m，安全矩形为 0.802 × 0.710 m |
| 前进、转向速度 | 0.6 m/s、1.2 rad/s，仿真动画用 |

消除区居中，车头到消除区近边的默认间距为 18.3 cm。pose 参考点和旋转中心暂按车身中心。车高仅用于记录车型，二维碰撞使用平面轮廓。

- 车体 x 轴向前、y 轴向左。`collector.gap` 独立于 `safety.body_margin`，省略时沿用外扩值。`lateral_offset` 为左右偏移，正值向左。
- 目前仅支持 `body_center` 参考点和旋转中心，其他值会报错。
- 车型选择优先级：显式 `robot_config` / `--robot-config` → 场景的 `custom.robot_config` → 默认车辆文件。场景引用的相对路径以场景文件所在目录为基准。
- 配置缺失或非法时明确报错，不回退为旧演示尺寸。
- 界面尺寸调整在本次会话的重置和场景切换中保留，并进入导出的 JSON；不会改写车辆文件。永久修改应编辑 `sim/config/robot.yaml`。

## Python SDK / 算法调用接口

本地 Python 包名为 `route_plan_sim`。导入 `Scene` 和 `PoseController` 不创建窗口，也不加载 IR-SIM 显示层。使用 `uv run --project sim python` 可直接调用；在其他 Python 环境中，可以通过该环境的 `python -m pip install -e /完整路径/route_plan/sim` 安装。

```python
import math
from route_plan_sim import Scene, PoseController

scene = Scene(scenario="front")  # 自动加载默认车辆配置
# 其他选项：Scene(scenario="random", seed=42, robot_config="/完整路径/robot.yaml")
controller = PoseController(scene)

# map 下的绝对目标位姿。Python API yaw 为弧度。
controller.set_goal(3.5, 4.5, math.pi / 2)
while controller.command is not None:
    controller.step(0.04)

state = scene.snapshot()
print(state["pose"], state["remaining"], state["failure"])
```

- `scene.pose`：当前 `(x, y, yaw)`，米、米、弧度。
- `scene.targets` / `scene.collected`：未消除 / 已消除目标，格式为 `{id: (x, y)}`。
- `(x, y)` 为圆心。`scene.target_diameter` / `scene.target_radius` 为统一物理尺寸，默认直径 0.10 m，来自场景的 `custom.target_diameter`，可通过 `Scene(target_diameter=...)` 覆盖。目标半径与车辆外扩距离独立，不可重复并入 `body_margin`。
- `scene.snapshot()`：返回位姿、剩余目标、已消除目标、失败原因、场景、种子、车辆尺寸和 `target_diameter`。
- `scene.clone()`：复制当前场景用于独立试算，不读取磁盘配置，不创建窗口；配置、目标、已消除点和轨迹不共享可变状态。
- `scene.add_target(x, y)`、`scene.remove_nearest(x, y)`：增删圆形目标；整个圆必须在地图内，不能接触安全轮廓。允许重叠和重合圆心，保留全部 ID。重叠连通组接触消除器后整组清除，仅相切不分组，清除前按全部成员圆检查碰撞。整批观测使用 `scene.add_targets(points)`，先验证全部成员再处理初始接触，避免依赖输入顺序。
- `scene.reset()`：恢复当前预设及同一随机种子的初始状态；不重新读取磁盘配置。重新初始化 `Scene` 才会读取文件中的新参数。
- `scene.set_dimensions(...)`：调整 `body_length`、`body_width`、`collector_width`、`collector_depth`、可选的 `body_margin`；非法值、越界或接触剩余点时拒绝整次更新。
- `controller.set_goal(x, y, yaw)`：下发目标；新指令替换旧指令。非法指令抛出 `ValueError`，旧指令保留。
- `controller.step(dt)`：推进指定仿真时间；`command is None` 表示无待执行动作，需同时检查 `scene.failure` 判断是否成功到达。
- `controller.paused = True/False`、`controller.stop()`：暂停 / 继续、取消动作。
- `scene.turn_to(degrees)`、`scene.forward(metres)`、`scene.advance(rotation=...)` 是底层同步几何动作接口；其中 `turn_to` 使用度，`advance` 的相对旋转使用弧度。常规算法对接优先使用目标位姿接口。

## 必须保持的运动与碰撞规则

1. 小车与目标使用同一 map 坐标系；界面 yaw 使用度，Python 目标位姿 API 使用弧度。0° 向右，90° 向上，逆时针为正。
2. 目标位置不能处于当前车头后半平面：`(goal_x-x)*cos(yaw) + (goal_y-y)*sin(yaw) >= 0`，正侧方允许。目标与当前位置相同时可以只调整朝向。
3. 执行顺序为原地转向目标位置 → 向前直线到达 `(x, y)` → 原地调整到最终 yaw。始终禁止倒车，不采用边走边转的模型。
4. 默认目标为直径 10 cm 的圆形实体，按真实 map 尺寸绘制。旧接触模式在前进、旋转时与圆接触（含相切）即消除。`stops` 模式途中不消除，到服务位姿后逐 ID 检查圆心必须在消除矩形内；仅圆边缘相切不算，不要求整个圆进入。重叠目标也不能顺带清除区域外圆心。碰撞始终使用完整半径。新停靠记录为 `pickup_rule="center"`，旧归档缺失字段按旧 `contact` 规则回放。
5. 碰撞按车身每侧外扩后的安全矩形与目标圆求交，圆与内部或边界接触、相切都不允许。安全轮廓和消除区也不能接触地图边界。
   正前方的外扩区同样属于碰撞区，不能作为消除区豁免。新场景使用 `custom.rule_version=3`，版本 2 归档的间距跟随外扩。旧归档缺少该字段时按版本 1 的原安装几何回放。
6. 检查整个运动过程，不能仅检查起点、终点或显示帧。连续接触时刻用圆心轨迹与矩形的直边、圆角偏置边界解析求解；不能用再次扩大长宽的矩形替代圆角，否则角部会误报。
7. 先被消除的目标不再触发后续碰撞；同一目标同时接触安全轮廓与消除区时，碰撞优先。
8. 碰撞后冻结在首次接触处，取消位姿指令剩余动作，重置后才能继续。随机场景不保证可解。

## 界面与开发验证

画布左键添加目标点、右键删除附近点，Shift + 左键设置目标位置（最终 yaw 取输入框）。W / 上箭头前进 0.25 米，A / D 或左右箭头原地转 15°，空格暂停，R 重置，Esc 停止。15° 只是快捷操作步长，不限制目标 yaw 的取值。

修改几何、控制或配置逻辑后运行上述单元测试；修改显示后还应导出截图检查中文字体、控件重叠、车身和安全轮廓的一致性。历史宽消除器测试使用 `sim/tests/fixtures/robot.yaml`，不可用它替换用户默认车型。

目标点由 `Scene` 管理并叠加绘制，未放进 IR-SIM 普通障碍物列表；IR-SIM 使用 external step 模式同步位姿。后续接规划器或 Nav2 时需要显式读取剩余点，不假定代价地图已经更新。

保持仿真和算法目录独立，不提交 `.venv/`、缓存、截图或其他 `sim/output/` 生成物。

纯算法 SDK 使用 `python planner/tools/package_sdk.py` 打包，默认输出 `sim/output/sdk/dist/`，不会重复跑完整回归；需要独立安装验证时加 `--verify`。SDK 不携带显示模块或 IR-SIM 依赖，只复制规划必需的几何内核到私有命名空间。维护原源文件和 `planner/sdk/`，不直接修改生成副本。

规划导出与回放必须保留目标直径。旧归档记录若没有目标尺寸字段，按其原来的点模型（直径 0）恢复，不能把旧点模型的成功轨迹或统计直接当成 10 cm 圆模型的验证。旧点解析测试可显式设置 `target_diameter=0`，正常初始化默认使用圆形实体。
