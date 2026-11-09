# 静态目标消除 baseline

跨工程集成可使用独立的 [纯算法 Python SDK](../docs/python-sdk.md)，无需安装仿真显示依赖，附带 Python 和 JSON 两种 example。

`--method stops` 优化拾取停靠次数。一个停靠位可同时覆盖多个目标，要求每个目标圆心都在消除矩形内。每段实际平移大于 5 cm，每次实际转向大于 1°，路程作为结果统计。目标只在明确的停靠位清除，使用 `result.pickup_stops` 执行和回放。中间位姿不计入拾取停靠数。命令示例 `uv run --project planner route-plan solve --method stops --scenario random --seed 42 --timeout 3 --gif sim/output/stops.gif`。

当前默认目标为直径 10 cm 的圆，候选位姿、几何回放及长度下界均考虑目标半径。自动窗口、GIF、快照与对比图按实际 map 尺寸绘制圆形目标。记录保留目标直径；未带尺寸的历史记录按原点模型回放。

启动时所有目标完整、准确可见，执行期间保持静态。当前实现通过任务搜索与运动连接器联合求解，并在仿真副本中完整回放后报告成功。保留可行性 baseline，并增加路径长度优化方法；两者都首先要求全部清除及运动、安全规则。优化版在有限预算内寻找更短路线，不要求或宣称全局最优。

算法实现与仿真独立。`TaskPlanner`、`MotionBackend` 接口允许替换任务搜索或运动规划方法，不把当前搜索器作为唯一技术路线。

当前默认为直径 10 cm 的圆形目标，消除区宽 35 cm、深 24 cm。规划按圆与矩形接触规则试算，支持圆边及圆角接触的原地消除；新模型回归见 [圆形目标适配说明](../docs/circle-target-adaptation.md)。

## 环境

从仓库根目录操作。可以使用独立 Conda 环境：

```bash
conda env create -f environment.yml
conda activate route-plan
python -m pip install -e ./sim -e ./planner
```

已有 `route-plan` 环境时直接激活。也可沿用仓库的 uv 工作流：

```bash
uv sync --project planner --locked
uv run --project planner route-plan solve --scenario front
```

`planner/uv.lock` 锁定 uv 环境依赖；Conda 环境通过两个本地可编辑包安装，关键库固定为 IR-SIM 2.12.0、OMPL 2.0.1。

## 自动演示与回放

以下命令假定已激活 Conda 环境；使用 uv 时，在命令前加 `uv run --project planner`。

```bash
# 先完整规划及验证，再在 IR-SIM 窗口自动执行
route-plan solve --scenario front --show
route-plan solve --scenario surround --show
route-plan solve --scenario random --seed 7 --show

# 在同样约束下继续优化路径长度，保留可行性初解作为保底
route-plan solve --method length --scenario random --seed 7 --timeout 3 --show

# 无窗口生成可复现记录、动画和最终截图
MPLBACKEND=Agg route-plan solve --scenario random --seed 42 \
  --output sim/output/baseline/random-42.json \
  --gif sim/output/baseline/random-42.gif \
  --snapshot sim/output/baseline/random-42-after.png

# 不重新搜索，使用记录中保存的完整车辆参数、场景和动作回放
route-plan replay sim/output/baseline/random-42.json --show

# 从保存的场景重新搜索；可对比另一套预算或禁用 OMPL
route-plan solve --input sim/output/baseline/random-42.json --no-ompl
```

窗口沿用仿真器中文界面。空格暂停或继续，Esc 取消整份排队计划；手动运动、修改点位、重置或应用尺寸时取消后续自动计划。第一阶段不会因手动修改自动重规划。

GIF 为加速展示，淡绿色叉号标记已消除点；绘图只读取 `Scene` 的位姿、轮廓和目标状态，不独立推算碰撞。原始 `route-sim` 手动入口继续可用。

## 当前算法

1. 为剩余目标生成接近方向与车辆位姿，目标应进入消除区，而非落在车身中心。
2. 优先尝试原地转向、前进、最终转向；必要时先在更远的中间位姿对准，再沿消除器方向接近。
3. 所有候选在 `Scene.clone()` 副本中通过 `PoseController` 执行，实时更新途中实际消除的所有点。
4. 深度优先搜索保留候选分支，死路时在内存中回溯，尝试其他接近方向或目标顺序。回溯不代表小车倒车。
5. 简单接近不足时，调用 OMPL RRT 搜索中间位姿；还保留有限次数的非消除移动。
6. 找到完整方案后，以 0.04 秒步长独立回放。只有没有碰撞且剩余目标为零，才返回 `success`。

候选排序参考距离与转向，只影响探索顺序，不代表已实现最短路径优化。当前使用有限方向采样、毫米/毫弧度量化的访问状态和有限搜索预算，不保证搜索完备。

OMPL 的每条边也使用转向、前进、转向的实际执行规则检查，不执行默认的边走边转插值。当前局部 RRT 保守地要求绕行部分不消除目标，在最后接近段允许消除并将真实剩余集合交回任务搜索。任意沿途消除的复杂路径可由后续运动后端扩展，当前不宣称固定 SE2 树已经表达目标删除状态。

OMPL 随机种子在每个进程第一次使用时初始化；同进程后续调用沿用随机流，时间预算也可能导致搜索结果变化。保存的完整目标位姿序列可独立回放，不依赖再次搜索得到相同路线。

## Python 接口

```python
from route_plan_sim import Scene
from route_plan_planner import PlannerOptions, StaticPlanner, replay

scene = Scene(scenario="random", seed=42)
planner = StaticPlanner(PlannerOptions(timeout=3.0))
result = planner.plan(scene)  # 不修改 scene
if result.status == "success":
    final = replay(scene, result.goals)  # 返回独立执行后的场景
    assert not final.targets and not final.failure
```

路径长度优化方法可单独使用，也可复用已有可行性初解：

```python
from route_plan_planner import LengthPlanner

options = PlannerOptions(timeout=3.0)
a = StaticPlanner(options).plan(scene)
b = LengthPlanner(options).plan(scene, initial=a)
# 成功的 A 会作为保底，B 返回的路线不会更长。
```

`--method feasibility` 为默认方法；`--method length` 对应 `LengthPlanner`。优化包含完整后续动作验证的路径捷径、消除顺序 2-opt、接近位姿重建和有限宽度的候选搜索。距离按车身中心的总平移路程计算，不加入转向或耗时惩罚。只有完整清除且安全的更短方案才会替换初解；优化预算耗尽时仍返回已有可行方案，`status` 保持 `success`，具体终止原因在 `optimization` 字段中。

`StaticPlanner(options, motion=..., fallback=...)` 支持注入其他连接器。接口定义见 `core.py` 和 `motion.py`。角度统一为弧度；每条下发的目标位置仍须符合车头前半平面规则，处理后方目标时先合法原地转向。

结果状态包括 `success`、`invalid_input`、`budget_exhausted`、`not_found`、`replay_failed`。未成功时 `goals` 不提供可执行完整计划；`diagnostic.partial_goals` 仅用于诊断最佳试算分支，不自动执行。`remaining` 表示实际回放剩余数，未执行时保留输入数量；`best_remaining` 表示搜索曾达到的最少剩余数。

## 验证

```bash
MPLBACKEND=Agg python -m unittest discover -s sim/tests -v
MPLBACKEND=Agg python -m unittest discover -s planner/tests -v

# 两个固定预设 + 100 个随机场景，每个场景 3 秒搜索预算
route-plan benchmark --seeds 0:100 --timeout 3 \
  --output sim/output/baseline/benchmark

# A/B：同一场景、同一可行性初解，B 利用剩余预算继续优化
route-plan compare --seeds 0:100 --timeout 3 --output sim/output/length-ab

# 精确选择十个随机种子，不额外加入固定预设
route-plan compare --seed-list 41,57,28,38,49,70,1,97,85,54 --random-only \
  --timeout 3 --output sim/output/circle-regression-10
```

A/B 对比会逐例保存 A、B 的完整输入和计划，并统计清除率、路程、改进比例和耗时。A 找到首个可行方案就停止；B 复用该方案，初解耗时计入同一搜索预算。最终安全回放可能增加墙钟耗时，`elapsed` 包含验证开销。详情与实测结果见 [路径长度优化与 A/B 验证](../docs/path-length-optimization.md)。

固定回归集在 `tests/fixtures/static_cases.json`，包含冻结的尺寸、地图、初始状态、预算与可行基准动作。除校验基准动作，还重新规划并独立回放；另有 OMPL 绕行、连续旋转碰撞、多个回放步长和记录重放测试。

2026-11-08 的历史点模型本机 baseline：49 项仿真测试、12 项规划器测试通过；两个默认预设及随机种子 0–99 共 102/102 个场景完成全部清除。每个随机场景有 40 个目标。这是旧点模型在该固定集合的一次实测，不能当作当前 10 cm 圆模型的成功率，不代表任意布局都存在解或当前算法能找到所有可行解。

验证摘要见 [`benchmarks/baseline-v1.json`](benchmarks/baseline-v1.json)，阶段说明见 [`docs/static-baseline.md`](../docs/static-baseline.md)。截图、动画、逐例完整记录位于被忽略的 `sim/output/`，不纳入版本管理。
