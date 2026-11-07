# Route Plan

带前置消除区域的小车任务规划实验。接收 map 坐标系中的目标位姿 `(x, y, yaw)`，先原地转向、向前行驶，再调整到目标朝向。禁止下发位于当前车头后方的目标位置；车身外扩后的安全轮廓不能接触尚未消除的目标点。

- [`sim/`](sim/README.md)：独立的 IR-SIM 场景、交互窗口和几何规则，包含自己的依赖与启动入口。
- [`sim/config/robot.yaml`](sim/config/robot.yaml)：默认车辆配置，每次初始化自动读取。
- 后续规划算法单独放置，通过仿真接口读取状态、执行转向和前进动作。

启动场景：

```bash
uv run --project sim route-sim
```
