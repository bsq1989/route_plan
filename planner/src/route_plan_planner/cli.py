"""Solve, replay and benchmark static scenes without creating a window by default."""

import argparse
from dataclasses import asdict
from importlib.metadata import version
from pathlib import Path
import platform
import sys

from route_plan_sim import Scene

from .core import PlannerOptions, StaticPlanner
from .length import LengthOptions, LengthPlanner
from .motion import replay
from .records import describe_scene, read_json, restore_scene, write_json


def _options(args):
    return PlannerOptions(timeout=args.timeout, max_expansions=args.max_expansions,
                          max_trials=args.max_trials, ompl=not args.no_ompl,
                          seed=args.planner_seed)


def _plan_record(scene, options, method="feasibility"):
    initial = describe_scene(scene)
    result = (LengthPlanner(options) if method == "length" else StaticPlanner(options)).plan(scene)
    payload = {"schema_version": 1, "planner": "length-beam-v1" if method == "length" else "static-dfs-v1", "python": platform.python_version(),
               "options": asdict(options), "initial": initial, "result": result.to_dict()}
    if method == "length":
        payload["length_options"] = asdict(LengthOptions())
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
    result, payload = _plan_record(scene, _options(args), args.method)
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
            result, payload = _plan_record(scene, options, args.method)
            name = f"{scenario}-{seed}"
            write_json(args.output / f"{name}.json", payload)
            row = {"case": name, "initial_count": len(scene.targets),
                   **{key: value for key, value in result.to_dict().items() if key not in ("goals", "final")}}
            rows.append(row)
            print(f"{name}: {result.status}, {result.elapsed:.3f}s, best_remaining={result.best_remaining}", flush=True)
    summary = {"planner": "length-beam-v1" if args.method == "length" else "static-dfs-v1", "options": asdict(options), "cases": rows,
               "total": len(rows), "success": sum(row["status"] == "success" for row in rows)}
    write_json(args.output / "summary.json", summary)
    print(f"success {summary['success']}/{summary['total']}; {args.output / 'summary.json'}")
    return 0 if summary["success"] == summary["total"] else 1


def _compare(args):
    start, stop = map(int, args.seeds.split(":"))
    if not 0 <= start < stop:
        raise ValueError("seeds must be a nonempty, nonnegative START:STOP range")
    options = _options(args)
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for scenario, seeds in (("front", [42]), ("surround", [42]), ("random", range(start, stop))):
        for seed in seeds:
            scene = Scene(scenario=scenario, seed=seed, robot_config=args.robot_config)
            a, record_a = _plan_record(scene, options)
            # Share exactly the same feasible incumbent, not a second randomized
            # solve. Its elapsed time consumes B's overall search allowance.
            b = LengthPlanner(options).plan(scene, initial=a)
            record_b = {**record_a, "planner": "length-beam-v1",
                        "length_options": asdict(LengthOptions()), "result": b.to_dict()}
            name = f"{scenario}-{seed}"
            write_json(args.output / f"A-{name}.json", record_a)
            write_json(args.output / f"B-{name}.json", record_b)
            paired = a.status == b.status == "success"
            row = {"case": name, "initial_count": len(scene.targets),
                   "a_status": a.status, "b_status": b.status,
                   "a_length": a.length if a.status == "success" else None,
                   "b_length": b.length if b.status == "success" else None,
                   "a_elapsed": a.elapsed, "b_elapsed": b.elapsed,
                   "saved_length": a.length - b.length if paired else None,
                   "saved_fraction": (1 - b.length / a.length) if paired and a.length > 0 else 0. if paired else None}
            rows.append(row)
            print(f"{name}: A={a.status} {a.length:.2f}m; B={b.status} {b.length:.2f}m; "
                  f"saved={row['saved_fraction']:.1%}" if paired else f"{name}: A={a.status}; B={b.status}", flush=True)
    paired = [row for row in rows if row["saved_length"] is not None]
    total_a = sum(row["a_length"] for row in paired)
    total_b = sum(row["b_length"] for row in paired)
    summary = {"schema_version": 1, "a_method": "static-dfs-v1", "b_method": "length-beam-v1",
               "options": asdict(options), "length_options": asdict(LengthOptions()),
               "shared_feasible_seed": True, "timeout_scope": "seed plus optimization search; final replay may add time",
               "total": len(rows), "a_success": sum(row["a_status"] == "success" for row in rows),
               "b_success": sum(row["b_status"] == "success" for row in rows), "paired_success": len(paired),
               "improved": sum(row["saved_length"] > 1e-8 for row in paired),
               "unchanged": sum(abs(row["saved_length"]) <= 1e-8 for row in paired),
               "worse": sum(row["saved_length"] < -1e-8 for row in paired),
               "a_total_length": total_a, "b_total_length": total_b,
               "total_saved_fraction": 1 - total_b / total_a if total_a else 0., "cases": rows}
    write_json(args.output / "summary.json", summary)
    print(f"A/B: success {summary['a_success']}/{summary['b_success']}; "
          f"improved {summary['improved']}; total saved {summary['total_saved_fraction']:.1%}")
    return 0 if summary["paired_success"] == summary["total"] and summary["worse"] == 0 else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description="静态目标消除 baseline：规划、逐步回放、批量验证")
    parser.add_argument("--version", action="version", version=f"%(prog)s {version('route-plan-planner')}")
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
    compare = commands.add_parser("compare", help="同场景同初解对比可行性 A 与路径长度优化 B")
    compare.add_argument("--seeds", default="0:100")
    compare.add_argument("--output", type=Path, default=Path("sim/output/length-ab"))
    for command in (solve, batch, compare):
        command.add_argument("--robot-config", type=Path)
        command.add_argument("--timeout", type=float, default=30.)
        command.add_argument("--max-expansions", type=int, default=2000)
        command.add_argument("--max-trials", type=int, default=30000)
        command.add_argument("--no-ompl", action="store_true")
        command.add_argument("--planner-seed", type=int, default=42)
    for command in (solve, batch):
        command.add_argument("--method", choices=("feasibility", "length"), default="feasibility")
    for command in (solve, play):
        command.add_argument("--show", action="store_true", help="打开 IR-SIM 窗口自动执行，空格暂停、Esc 停止")
        command.add_argument("--snapshot", type=Path)
        command.add_argument("--gif", type=Path)
    args = parser.parse_args(argv)
    try:
        return {"solve": _solve, "replay": _replay, "benchmark": _benchmark, "compare": _compare}[args.command](args)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
