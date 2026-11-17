# Tracer 2.0 + Piper X · MuJoCo

官方 Tracer 2.0 提供裸底盘和 Piper X 整车两个场景。两份都支持差速行驶，整车另有六轴和夹爪控制。工程使用单独的 uv 环境。

| 场景 | 文件 |
| --- | --- |
| 官方 Tracer 2.0 裸底盘 | [model/scene_tracer.xml](model/scene_tracer.xml) |
| 官方 Tracer 2.0 底盘 + Piper X（默认） | [model/scene.xml](model/scene.xml) |
| 历史项目版底盘 + Piper X | [model/scene_project.xml](model/scene_project.xml) |

## 启动和重建

可以直接用 MuJoCo 打开对应的 XML。场景共用 `model/meshes/` 里的网格和贴图，移动场景时把整个 `model/` 目录一起带走。也可在仓库根目录启动查看器。

```bash
uv run --project mujoco-dev python -m mujoco.viewer --mjcf=mujoco-dev/model/scene.xml
uv run --project mujoco-dev python -m mujoco.viewer --mjcf=mujoco-dev/model/scene_tracer.xml
```

整车联动演示会自动执行直行、转向以及移动中开合夹爪，并停在结束姿态。macOS 用以下命令运行，Linux 将 `mjpython` 换成 `python`。

```bash
uv run --project mujoco-dev mjpython mujoco-dev/demo_viewer.py
```

重新拉取和生成模型：

```bash
./mujoco-dev/fetch_assets.sh
uv run --project mujoco-dev --group build python mujoco-dev/build_model.py
uv run --project mujoco-dev python mujoco-dev/check_model.py
```

拉取脚本从 `dgx@192.168.112.108` 上的 `d252837c84bb` 容器读取 `/workspace/tracer_piper`，并从 AgileX GitHub 仓库下载固定版本的官方 Tracer 2.0 URDF 和 STL。SSH 参数可通过 `TRACER_SSH_HOST` 和 `TRACER_CONTAINER` 覆盖。原始资产保存在 `source/`，生成的 MJCF 与网格在 `model/`，检查报告在 `output/`。Git 忽略这三个本地目录。

## 模型来源

- 两个官方场景使用 AgileX Tracer 2.0 的车壳和六个轮子网格，车壳碰撞采用其凸包，两个驱动轮外观随驱动关节转动。官方 STL 只有几何，车壳配色、灯罩和标识贴图按官网产品图复刻，两份场景共用。[AgileX 底盘 URDF](https://github.com/agilexrobotics/ugv_gazebo_sim/blob/27633a956c845903ee630538afeb17fe70afdd84/tracer/tracer2_description/urdf/tracer2_description.urdf) · [AgileX 产品图](https://global.agilex.ai/products/tracer-2-0)
- 项目版底盘、整车的上装箱、雷达和车头相机外壳来自 `mobile_manipulator2_chassis_camera_hole_aligned.urdf`。整车采用移除显示面板、竖杆和杆顶相机后的构型。前进方向为 `base_link` 的 +X，左侧为 +Y，Piper X 安装点为 `(0.118, 0, 0.100)` m。
- 整车 URDF 内的手臂为旧 Piper。当前 Isaac 场景使用 `official_piper_x/piper_x_gripper_d435.usd`，转换脚本从该资产读取 Piper X 的关节变换、限位、惯量、外观、碰撞和腕部相机，并把 USD 分面颜色转换成 MuJoCo 贴图。导入时 OpenUSD 可能报告占位 `world/visuals` 缺失，该占位节点在整车场景中由浮动底盘取代。
- 动力学沿用 Isaac 原型的 55 kg 裸车质量、0.5174 m 有效轮距和 0.0625 m CAD 轮半径。驱动轮使用圆柱接触，支撑轮使用四个无摩擦球形接触点。项目版显示合并轮子外观的 CAD 底盘，并使用长方体车壳碰撞体。
- 参考 Isaac 原型关闭了整车自碰撞。底盘驱动、机械臂伺服和轮胎摩擦是仿真参数。腕部 `wrist_d435` 是普通 MuJoCo 渲染相机，车载相机目前保留外壳和安装坐标。深度噪声、悬挂和实车动力学标定未建模。

## 控制和验证

可在查看器的 Control 面板调整执行器，也可通过 `data.actuator(name).ctrl[0]` 按名称控制。左右轮 `left_wheel_joint`、`right_wheel_joint` 接收 rad/s，`joint1` 至 `joint6` 接收 rad，`gripper_joint1`、`gripper_joint2` 接收 m，开夹时目标分别为正、负。底盘速度到左右轮速度的转换示例见 `check_model.py` 中的 `set_base_velocity`。

无窗口检查覆盖静止、直行、停车、原地转向，以及底盘、六轴和夹爪同时运动。结果保存在 `output/check.json`。

## 布面隔离围栏

[围栏预览场景](props/banner_barrier/preview.xml)可直接在 MuJoCo 打开。[围栏模块](props/banner_barrier/banner_barrier.xml)包含固定的圆底座、立柱、横杆和带双面 Visincept 原图的布面。移动模块时把同目录的 `visincept_banner.png` 一起带走。

柱中心距 1480 mm、总高 900 mm、圆底座直径 320 mm。按参考图估算，立柱直径 65 mm、下横杆中心离地 120 mm、杆下净空 110 mm。模型原点在两柱中间的地面上，沿局部 X 轴展开。可参考预览 XML 用 `asset/model` 和 `attach` 装入现有场景。每次拼接用独立的前缀，位置和转角可通过外层 `frame` 设置。

[围栏试验场景](scenes/barrier_arena.xml)用 10 段护栏和 10 根共用立柱围成闭合矩形。长边 3 段、短边 2 段，按立柱中心线计为 4.44 × 2.96 m。官方 Tracer 2.0 裸底盘从场地中央起步。场景引用生成的 `model/scene_tracer.xml`，可直接用 MuJoCo 打开。

## ROS 2 Jazzy 容器

从仓库根目录执行以下命令。容器以无窗口模式运行 MuJoCo 物理引擎和 ROS 2 差速控制器，直接加载本机现有的 `scenes/barrier_arena.xml` 和 `model/`。在新检出的仓库中，如果 `model/` 尚未生成，先按上文准备模型。

```bash
docker build -t route-plan-mujoco:jazzy mujoco-dev/ros2
docker run -d --init --name route-plan-mujoco-ros2 \
  --mount "type=bind,source=$PWD/mujoco-dev,target=/workspace/mujoco-dev,readonly" \
  route-plan-mujoco:jazzy
```

控制器启动后，下面的指令会以 10 Hz 向仿真底盘发送 20 条前进速度消息。速度超时为 0.5 s，常规启动时车体前向速度下限为 0。

```bash
docker exec route-plan-mujoco-ros2 /ros_entrypoint.sh \
  ros2 topic pub --times 20 -r 10 /sim/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.2}}}'
```

`/sim/odom` 是轮式里程计，控制器发布 `odom → base_link`。MuJoCo 的车身真值独立发布在 `/sim/ground_truth/odom`。查看启动和控制器日志可用 `docker logs route-plan-mujoco-ros2`。

### 网页手动驾驶

网页模式下，MuJoCo 的物理和 OpenGL 渲染由 Mac 上的原生进程运行。Docker 运行 ROS 2、差速控制器和网页服务，通过 ROS 关节话题桥接左右轮速度指令与反馈。本地两个服务都绑定到回环地址。

在 Mac 上启动 Docker Desktop，准备 npm、uv 和上文的模型，然后从仓库根目录执行一键启动命令。

```bash
./mujoco-dev/ros2/start_web.sh
```

脚本准备前端和 Python 环境，启动 ROS 2 容器和 macOS 后台 MuJoCo，并等待两个服务就绪。同一份网页仿真正在运行时，重复执行会复用进程和当前车位。若要从常规容器模式切换，先停止并移除旧的同名容器。

打开 [本机控制页](http://127.0.0.1:8765/)。左侧将 MuJoCo 摄像机画面上传为 WebGL 纹理，右侧显示方向按钮和车身真值位姿。画面服务使用 8767 端口，只保留最新一帧，车和相机静止时复用纹理。

场景默认从正上方俯视。拖动旋转相机，按住 Shift 拖动或用右键拖动平移，滚轮或触控板捏合可在 1× 到 2.5× 之间缩放，双击恢复默认俯视。长按方向按钮、↑ / ← / → 或 W / A / D 控制前进和左右原地转向。松开、切出页面或按 Esc 停车，指令中断时也会自动停车。

在控制面板的「场景配置」区点击「加载消除区」，可导入 D405 的 `d405_mask_union_baselink.json`。场景用 `rings[].points_m` 的原始 XY 轮廓填充半透明叠层，随 MuJoCo 车身真值的 XY 和航向移动，保持在地面上方 5 mm。Chrome 显示「自动跟踪」时，保存源文件会更新叠层，临时无效的文件会保留上一版轮廓。刷新网页后重新选择文件。浏览器仅支持普通文件选择时，可以再次选择文件来加载新版。

[MuJoCo Python 查看器文档](https://mujoco.readthedocs.io/en/stable/python.html#interactive-viewer)
