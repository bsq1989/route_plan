# 独立 2D 仿真场景

基于 IR-SIM 2.12.0 的中文俯视交互窗口，用于摆点、手动运动、观察消除和碰撞。静态自动规划与自动执行演示位于独立的 [`planner/`](../planner/README.md)，仿真包自身不包含任务策略，也不依赖 ROS。

界面自动选择系统已有的中文字体（macOS 优先苹方，Windows 优先微软雅黑，Linux 支持 Noto Sans CJK SC 等）；如果系统没有中文字体，请先安装中文字体再启动。

## 启动

从仓库根目录运行（安装了 [uv](https://docs.astral.sh/uv/) 即可，依赖会自动安装）：

```bash
uv run --project sim route-sim
uv run --project sim route-sim --scenario surround
uv run --project sim route-sim --scenario random --seed 123
uv run --project sim route-sim --robot-config /完整路径/robot.yaml
```

也可以进入 `sim/` 后运行 `uv run route-sim`。首次运行需要下载 Python 依赖；交互模式需要桌面图形环境。依赖锁定在本目录的 `uv.lock`。

## 默认车辆配置

每次初始化自动读取 [`config/robot.yaml`](config/robot.yaml)，与运行命令所在目录无关。当前车辆参数：

| 参数 | 值 |
| --- | --- |
| 车身长度 / 宽度 / 高度 | 0.702 / 0.610 / 0.169 米 |
| 消除区横向宽度 / 向前深度 | 0.190 / 0.090 米 |
| 每侧安全外扩 | 0.050 米，安全矩形为 0.802 × 0.710 米 |
| 前进速度 / 转向速度 | 0.6 米/秒、1.2 弧度/秒，用于仿真动画 |

消除区默认居中安装。`collector.gap` 设置车头到消除区近边的间距，默认 18.3 cm，自定义车型省略时使用 `safety.body_margin`。`lateral_offset` 设置左右偏移，正值向左。当前支持车身中心作为位姿参考点和旋转中心。车高仅作车型记录。

地图和初始位姿保存在 `src/route_plan_sim/world.yaml`，车辆尺寸、安装参数、安全外扩和速度集中在车辆文件中。`--robot-config` 可显式选择另一车型；自定义场景也可用 `custom.robot_config` 引用相对于场景文件的车辆配置。显式命令行参数优先。配置文件缺失或无效会报错，不会回退为演示尺寸。

## 操作

| 操作 | 效果 |
| --- | --- |
| 右侧场景选择 | 前方点群 / 四周包围 / 随机分布 |
| 尺寸设置 + 应用尺寸 | 设置车身长宽、消除区宽度和深度、车身外扩距离（米），立即更新几何判定，并停止当前动作 |
| 目标 `x、y、yaw` + 前往目标位姿 | 下发 map 下的绝对目标位姿；x、y 单位米，界面 yaw 单位度，0° 向右、90° 向上、逆时针为正 |
| Shift + 画布左键 | 使用点击位置作为目标 x、y，最终 yaw 取右侧输入框的值 |
| W / 上箭头 | 前进 0.25 m |
| A / 左箭头；D / 右箭头 | 原地左转 / 右转 15° |
| 空格 / 暂停 | 暂停或继续当前动作 |
| Esc | 取消当前动作，在当前位置停下 |
| R / 重置 | 恢复当前预设场景，包括同一随机种子对应的点位 |
| 画布左键；右键 | 添加目标点；删除附近未消除点（运动时请先按 Esc） |
| 保存截图和状态 | 保存画面和状态到 `sim/output/`，JSON 包含完整尺寸配置 |

蓝色为车身，绿色为前方消除区，橙色为剩余点，浅橙色虚线框为外扩后的安全轮廓，蓝色虚线框为目标位姿。被消除的点从画面消失；碰撞后车身显示红框，冻结在首次接触的位置，需要重置。

## 圆形目标

默认每个目标为**直径 0.10 m（10 cm）、半径 0.05 m 的圆**。`scene.targets` 中的 `(x, y)` 表示圆心，现有 SDK 的增删接口保持可用。直径位于场景文件 `src/route_plan_sim/world.yaml` 的 `custom.target_diameter`，也可用 `Scene(target_diameter=0.1)` 覆盖，与车辆参数分开保存。

- 目标圆触碰车身外扩后的安全矩形就算碰撞，包括边、角和恰好相切；圆心仍在矩形外，也可能已经碰撞。
- 消除区与目标圆发生任何接触（包括相切）即消除，不要求圆心或整个圆进入；同时接触安全矩形和消除区时仍按碰撞处理。
- 前进和旋转都检查连续接触时刻，包含两帧间短暂接触和旋转角部相切。角部使用圆到矩形的真实距离，不将其扩大为方形。
- 新增目标要求整个圆在地图内、与安全矩形分离。允许目标重叠或圆心重合，重叠连通组在消除器接触任意成员时一次清除，保留每个 ID；仅相切不分组，清除前仍检查全部成员的碰撞。`add_targets(points)` 原子添加整批观测，先验证全部成员再处理初始接触。随机场景仍按圆尺寸生成不重叠目标。
- 消除区间距和车身外扩可以独立配置。整个外扩矩形都参与碰撞检查，消除区与安全轮廓重叠时碰撞优先。新规则版本为 `custom.rule_version=3`。
- `custom.collection_mode` 默认 `contact`，用于原运动接触消除模式。停靠优化使用 `at_stop`：途中及旋转不删除目标，到位后调用 `scene.collect_at_stop()` 清除同一位姿实际覆盖的目标，返回本批内部 ID。规划回放传入 `pickup_stops`，不能省略拾取检查点。
- 新停靠模式为 `pickup_rule="center"`，逐目标要求圆心在消除矩形内；圆边缘相切、部分相交但圆心在外都不清除，重叠组也不例外。碰撞继续按完整圆检查。旧记录恢复时保留原 `contact` 覆盖规则。
- 目标按 map 米制半径绘制，缩放、导出图片或播放自动规划时均保持真实尺寸。JSON 快照与规划记录保留直径，场景副本和重置保留该尺寸。

旧归档记录未保存直径时按原点模型恢复，避免改变历史轨迹含义。`target_diameter=0` 仅用于这种兼容和点几何回归测试；正常新场景默认使用 10 cm 圆。历史点模型 benchmark 不能代替圆形目标验证。

## map 位姿控制

小车位姿和目标位姿均使用同一 map 坐标系。指令可给任意连续角度，不受 15° 快捷步长限制，右侧输入框编辑完成后点击“前往目标位姿”统一下发。新目标会替换尚未完成的旧目标。

执行顺序：先原地转向目标位置，向前直线到达 `(x, y)`，再原地调整到最终目标 yaw。目标与当前位置相同时只调整朝向。给定的 yaw 表示到达后的朝向，不要求它与当前位置到目标位置的连线方向一致。

目标位置必须位于当前车头的前半平面（正侧方允许）：`(goal_x-x)*cos(yaw) + (goal_y-y)*sin(yaw) >= 0`。这里 yaw 是当前车身朝向，拒绝后方目标时不会修改当前指令。小车始终不倒车。

当前控制器仅负责转向与直线执行，避障和消除任务由独立规划器生成目标位姿序列。执行途中若安全轮廓触点或越界，会立即停止整条位姿指令，不会继续移动或调整末端朝向。

## 尺寸与规则

在窗口右侧的“尺寸设置”中输入车身长度、车身宽度、消除区宽度、消除区深度和车身外扩距离，再点击“应用尺寸”。深度是消除区从车头向前伸出的距离；改变车身长度时，消除区会保持紧贴车头。

外扩值为车身每一侧增加的距离，安全矩形长度 = 车长 + 2 × 外扩值，宽度 = 车宽 + 2 × 外扩值。点只要接触安全矩形内部或边界，就算碰撞，不必等到接触实体车身。默认外扩 0.10 m，可设为 0；外扩不改变实体车身和消除器的尺寸或位置。

修改输入框期间不会改变场景，点击应用后才一起生效。应用会停止当前动作，保留小车位置、轨迹和现有目标点。新安全轮廓若接触未消除点、安全轮廓或消除区在当前位置或初始位置越界、尺寸不是大于零的有限数值、外扩值不是非负有限数值，会拒绝应用并保留原尺寸。扩大消除区后，当前落入其中的点会立即消除。

应用后的尺寸在本次会话的重置和场景切换中保留，并写入导出的状态 JSON。若新尺寸不适用于某个固定点位预设，切换时会提示并保留原场景，可以缩小车身或选择随机分布。界面临时调整不会改写车型文件；要修改下次初始化的默认值，请编辑 [`config/robot.yaml`](config/robot.yaml)。

默认场地 12 × 9 m，初始位姿 `(3, 4.5, 0)`；车身和消除区使用上面的车辆配置。

旋转中心暂定为矩形车身中心。小车前进和原地旋转分开执行，不模拟打滑、惯性、传感器噪声。

- 目标为直径 10 cm 的圆形实体，显示尺寸使用真实 map 单位。
- 消除区在前进和旋转时都有效，与目标圆接触就消除；目标圆触碰车身安全轮廓的边界也算碰撞。
- 按运动中首次接触的时间处理事件，先被消除的点不再触发后续安全轮廓碰撞；同一点同时接触安全轮廓与消除区时按碰撞处理。
- 使用矩形直边与圆角偏置边界的解析交点检查整个动作，包含旋转角部相切、首尾都安全但中途碰撞的情况。
- 安全轮廓和消除区都不能接触地图边界。随机点避开初始安全轮廓和消除区，但不保证场景一定可解。
- 三种预设点位以初始机器人位姿为参考；自定义更小场地、较大车身后，预设点若越界会明确报错。

## 无窗口验证与截图

```bash
uv run --project sim python -m unittest discover -s sim/tests -v
uv run --project sim route-sim --snapshot sim/output/front.png
uv run --project sim route-sim --scenario surround --snapshot sim/output/surround.png
uv run --project sim route-sim --distance 1.5 --snapshot sim/output/collected.png
```

`--heading`、`--distance` 可在打开窗口或导出截图之前依次执行一次转向和前进。导出 PNG 时同时生成同名 JSON，不会弹窗。

## 静态实测围栏

从仓库根目录运行。

```bash
uv run --project sim route-sim --config sim/config/measured-fence.yaml --scenario random
uv run --project planner route-plan solve --config sim/config/measured-fence.yaml --scenario random --seed 41 --method stops
```

设 `custom.fence_enabled: true` 使用 `custom.fence` 的多边形边界。设为 `false` 时，保留原顶点并按 `world.width`、`world.height` 和 `world.offset` 定义的矩形运行。旧配置只要有顶点，省略开关时仍启用围栏。默认 `world.yaml` 使用原来的矩形。

顶点按围栏边界顺序排列，坐标为 map 下的米。实测配置直接使用 Gateway 回归中已内缩 16 cm 的有效顶点。车身安全轮廓和消除器在直行与原地转向时都检查连续触边。用 `PlanningRequest.from_scene(scene, method="stops")` 可把同一份边界和场景快照传给 SDK。

## 给后续算法的接口

几何规则在 `scene.py`，导入时不创建窗口，也不加载 IR-SIM：

```python
from route_plan_sim import Scene, PoseController

scene = Scene(scenario="random", seed=42)
# Scene() 已默认加载 sim/config/robot.yaml。
# 选择其他车型：Scene(robot_config="/完整路径/robot.yaml")。
# scene.target_diameter == 0.1；targets 中的坐标为圆心。
state = scene.snapshot()  # pose: x, y, theta（弧度）；剩余 / 已消除点；失败状态
controller = PoseController(scene)
controller.set_goal(4.0, 5.0, 1.57)  # map 目标 x、y、yaw；API yaw 使用弧度
while controller.command is not None:
    controller.step(0.04)
print(scene.snapshot())
```

独立算法包可通过可编辑依赖 `pip install -e ./sim` 使用此接口，当前 `planner/` 已采用这一方式。`scene.clone()` 可复制当前场景用于独立试算，不重新读取配置，也不共享可变状态。`advance(rotation=...)` 还支持带方向的相对旋转，单位弧度，可执行完整一圈。

`app.py` 负责动画和手动控制，使用 IR-SIM 官方的 external step 模式，把规则引擎的位姿同步给 IR-SIM。目标点由规则引擎管理并叠加绘制，不放进 IR-SIM 普通障碍物列表，以便按先后顺序处理消除和车身碰撞。后续导航适配需要显式读取 `scene.targets`，当前没有接入 Nav2 或代价地图。
