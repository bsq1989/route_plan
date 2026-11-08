"""Solve, replay and benchmark static scenes without creating a window by default."""

import argparse
from dataclasses import asdict
from pathlib import Path
import platform
import sys

from route_plan_sim import Scene

from .core import PlannerOptions, StaticPlanner
from .motion import replay
from .records import describe_scene, read_json, restore_scene, write_json


def _options(args):
    return PlannerOptions(timeout=args.timeout, max_expansions=args.max_expansions,
                          max_trials=args.max_trials, ompl=not args.no_ompl,
                          seed=args.planner_seed)


def _plan_record(scene, options):
    initial = describe_scene(scene)
    result = StaticPlanner(options).plan(scene)
    payload = {"schema_version": 1, "planner": "static-dfs-v1", "python": platform.python_version(),
               "options": asdict(options), "initial": initial, "result": result.to_dict()}
    return result, payload


def _render(args, scene, goals):
    if args.gif or args.snapshot or args.show:
        from .visualize import save_animation, save_snapshot, show
        if args.gif:
            final = save_animation(scene, goals, args.gif)
            if final.failure or final.targets:
                raise ValueError("Animation replay did not clear the scene")
        if args.snapshot:
            save_snapshot(replay(scene, goals), args.snapshot)
        if args.show:
            show(scene, goals)


def _solve(args):
    if args.input:
        record = read_json(args.input)
        scene = restore_scene(record.get("initial", record))
    else:
        scene = Scene(config=args.config, scenario=args.scenario, seed=args.seed,
                      robot_config=args.robot_config)
    result, payload = _plan_record(scene, _options(args))
    write_json(args.output, payload)
    print(f"{result.status}: {len(scene.targets)} -> {result.remaining}; "
          f"{result.elapsed:.3f}s; {result.length:.2f}m; "
          f"backtracks={result.backtracks}; record={args.output}")
    if result.status == "success":
        _render(args, scene, result.goals)
        return 0
    print(result.reason, file=sys.stderr)
    return 1


def _replay(args):
    record = read_json(args.record)
    if record.get("schema_version") != 1:
        raise ValueError("Unsupported plan record schema")
    if record["result"]["status"] != "success":
        raise ValueError("Only successful plans can be replayed")
    scene = restore_scene(record["initial"])
    goals = [tuple(goal) for goal in record["result"]["goals"]]
    final = replay(scene, goals)
    if final.failure or final.targets:
        raise ValueError(f"Replay failed: {final.failure}; remaining={len(final.targets)}")
    print(f"replay success: {len(scene.targets)} -> 0")
    _render(args, scene, goals)
    return 0


def _benchmark(args):
    start, stop = map(int, args.seeds.split(":"))
    if not 0 <= start < stop:
        raise ValueError("seeds must be a nonempty, nonnegative START:STOP range")
    options = _options(args)
    rows = []
    args.output.mkdir(parents=True, exist_ok=True)
    for scenario, seeds in (("front", [42]), ("surround", [42]), ("random", range(start, stop))):
        for seed in seeds:
            scene = Scene(scenario=scenario, seed=seed, robot_config=args.robot_config)
            result, payload = _plan_record(scene, options)
            name = f"{scenario}-{seed}"
            write_json(args.output / f"{name}.json", payload)
            row = {"case": name, "initial_count": len(scene.targets),
                   **{key: value for key, value in result.to_dict().items() if key not in ("goals", "final")}}
            rows.append(row)
            print(f"{name}: {result.status}, {result.elapsed:.3f}s, best_remaining={result.best_remaining}", flush=True)
    summary = {"planner": "static-dfs-v1", "options": asdict(options), "cases": rows,
               "total": len(rows), "success": sum(row["status"] == "success" for row in rows)}
    write_json(args.output / "summary.json", summary)
    print(f"success {summary['success']}/{summary['total']}; {args.output / 'summary.json'}")
    return 0 if summary["success"] == summary["total"] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description="静态目标消除 baseline：规划、逐步回放、批量验证")
    commands = parser.add_subparsers(dest="command", required=True)
    solve = commands.add_parser("solve", help="规划静态完整观测场景")
    solve.add_argument("--scenario", choices=("front", "surround", "random"), default="front")
    solve.add_argument("--seed", type=int, default=42)
    solve.add_argument("--config", type=Path)
    solve.add_argument("--input", type=Path, help="从完整场景记录或之前的运行记录重新规划")
    solve.add_argument("--output", type=Path, default=Path("sim/output/baseline/plan.json"))
    play = commands.add_parser("replay", help="按保存的尺寸和动作回放，不重新搜索")
    play.add_argument("record", type=Path)
    batch = commands.add_parser("benchmark", help="运行两个预设及一批随机种子，逐个保存复现记录")
    batch.add_argument("--seeds", default="0:100", help="START:STOP，右端不含")
    batch.add_argument("--output", type=Path, default=Path("sim/output/baseline/benchmark"))
    for command in (solve, batch):
        command.add_argument("--robot-config", type=Path)
        command.add_argument("--timeout", type=float, default=30.)
        command.add_argument("--max-expansions", type=int, default=2000)
        command.add_argument("--max-trials", type=int, default=30000)
        command.add_argument("--no-ompl", action="store_true")
        command.add_argument("--planner-seed", type=int, default=42)
    for command in (solve, play):
        command.add_argument("--show", action="store_true", help="打开 IR-SIM 窗口自动执行，空格暂停、Esc 停止")
        command.add_argument("--snapshot", type=Path)
        command.add_argument("--gif", type=Path)
    args = parser.parse_args(argv)
    try:
        return {"solve": _solve, "replay": _replay, "benchmark": _benchmark}[args.command](args)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
