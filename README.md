# Route Plan

当前版本：[v0.2.0 静态路径长度优化版](docs/releases/v0.2.0.md)。

带前置消除区域的小车任务规划实验。接收 map 坐标系中的目标位姿 `(x, y, yaw)`，先原地转向、向前行驶，再调整到目标朝向。禁止下发位于当前车头后方的目标位置；车身外扩后的安全轮廓不能接触尚未消除的目标点。

- [`sim/`](sim/README.md)：独立的 IR-SIM 场景、交互窗口和几何规则，包含自己的依赖与启动入口。
- [`sim/config/robot.yaml`](sim/config/robot.yaml)：默认车辆配置，每次初始化自动读取。
- [`planner/`](planner/README.md)：静态完整观测的自动清除 baseline，包含任务搜索、OMPL 绕行、自动演示与回放验证。
- [目标消除规划与分阶段验证计划](docs/planning-roadmap.md)：从静态完整准确观测开始，逐步验证车头方向加点和随机加点。
- [第一阶段静态规划 baseline](docs/static-baseline.md)：当前实现范围、验证结果与能力边界。
- [路径长度优化与 A/B 验证](docs/path-length-optimization.md)：在全部清除及安全约束下优化路程，对比可行性 baseline。

启动场景：

```bash
uv run --project sim route-sim
```

运行静态自动清除演示：

```bash
uv run --project planner route-plan solve --scenario random --seed 42 --show

# 路径长度优化版
uv run --project planner route-plan solve --method length --scenario random --seed 42 --timeout 3 --show
```
