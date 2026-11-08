# Route Plan

当前交付：[SDK v0.2.1 圆心覆盖停靠修正版](docs/releases/sdk-v0.2.1.md)。历史规划器版本：[v0.2.0 静态路径长度优化版](docs/releases/v0.2.0.md)。当前默认使用直径 10 cm 圆形目标和统一的车身外扩碰撞规则。

带前置消除区域的小车任务规划实验。接收 map 坐标系中的目标位姿 `(x, y, yaw)`，先原地转向、向前行驶，再调整到目标朝向。禁止下发位于当前车头后方的目标位置；车身外扩后的安全轮廓不能接触尚未消除的圆形目标（默认直径 10 cm）。

- [`sim/`](sim/README.md)：独立的 IR-SIM 场景、交互窗口和几何规则，包含自己的依赖与启动入口。
- [`sim/config/robot.yaml`](sim/config/robot.yaml)：默认车辆配置，每次初始化自动读取。
- [`planner/`](planner/README.md)：静态完整观测的自动清除 baseline，包含任务搜索、OMPL 绕行、自动演示与回放验证。
- [现行技术方案](docs/technical-design.md)：v0.2.0 的系统结构、任务搜索、运动规划、长度优化与验证机制。
- [圆形目标规划适配](docs/circle-target-adaptation.md)：直径 10 cm 的接触规则、接近动作与十组静态回归。
- [系统输入输出与集成接口](docs/integration-io.md)：输入字段、单位、规划输出、执行语义及最小对接示例。
- [纯算法 Python SDK](docs/python-sdk.md)：可独立安装的源码包、wheel、依赖清单和跨工程调用 example。
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
