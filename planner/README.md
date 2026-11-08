# 静态目标消除 baseline

启动时所有目标完整、准确可见，执行期间保持静态。当前实现通过任务搜索与运动连接器联合求解，并在仿真副本中完整回放后报告成功。当前优先保证全部清除及运动、安全规则；后续优化目标为满足这些约束后的总行驶路径最短。

算法实现与仿真独立。`TaskPlanner`、`MotionBackend` 接口允许替换任务搜索或运动规划方法，不把当前搜索器作为唯一技术路线。

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

`StaticPlanner(options, motion=..., fallback=...)` 支持注入其他连接器。接口定义见 `core.py` 和 `motion.py`。角度统一为弧度；每条下发的目标位置仍须符合车头前半平面规则，处理后方目标时先合法原地转向。

结果状态包括 `success`、`invalid_input`、`budget_exhausted`、`not_found`、`replay_failed`。未成功时 `goals` 不提供可执行完整计划；`diagnostic.partial_goals` 仅用于诊断最佳试算分支，不自动执行。`remaining` 表示实际回放剩余数，未执行时保留输入数量；`best_remaining` 表示搜索曾达到的最少剩余数。

## 验证

```bash
MPLBACKEND=Agg python -m unittest discover -s sim/tests -v
MPLBACKEND=Agg python -m unittest discover -s planner/tests -v

# 两个固定预设 + 100 个随机场景，每个场景 3 秒搜索预算
route-plan benchmark --seeds 0:100 --timeout 3 \
  --output sim/output/baseline/benchmark
```

固定回归集在 `tests/fixtures/static_cases.json`，包含冻结的尺寸、地图、初始状态、预算与可行基准动作。除校验基准动作，还重新规划并独立回放；另有 OMPL 绕行、连续旋转碰撞、多个回放步长和记录重放测试。

2026-11-08 的本机 baseline：49 项仿真测试、12 项规划器测试通过；两个默认预设及随机种子 0–99 共 102/102 个场景完成全部清除。每个随机场景有 40 个目标。这是该固定集合的一次实测，不代表任意布局都存在解或当前算法能找到所有可行解。

验证摘要见 [`benchmarks/baseline-v1.json`](benchmarks/baseline-v1.json)，阶段说明见 [`docs/static-baseline.md`](../docs/static-baseline.md)。截图、动画、逐例完整记录位于被忽略的 `sim/output/`，不纳入版本管理。
