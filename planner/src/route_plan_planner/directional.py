"""Run and export a reproducible directional pickup episode."""

import argparse
from dataclasses import asdict
from functools import partial
import math
from pathlib import Path

from route_plan_sim.directional import DirectionalScene, load_config
from route_plan_sim.scene import normalize

from .api import PlanningRequest, plan
from .direction_policy import CoveragePolicy
from .records import write_json


def run_case(name, seed=42, config_path=None, *, policy=CoveragePolicy):
    config = load_config(config_path)
    case = config["custom"]["directional"]["cases"][name]
    reference = config["custom"].get("robot_config")
    robot_config = (Path(config_path).resolve().parent / reference) if config_path and reference else None
    sim = DirectionalScene(config, name, seed, robot_config, planner_request=PlanningRequest.from_scene)
    agent = policy(sim, plan)
    frame = agent.capture()
    if case.get("probe_initial_heading") and frame["detections"]:
        success, _ = agent.pickup(frame)
        if not success:
            frame = agent.approach(frame["detections"][0], frame)
            agent.pickup(frame)
        if not sim.all_collected:
            frame = agent.capture()
    reason, tried = "round_limit", set()
    last_failure = None
    for _ in range(sim.regions.maximum_rounds):
        if sim.all_collected:
            break
        start_xy = sim.pose[:2]
        ok, latest_reason = agent.evaluate(frame, tried)
        if not ok and latest_reason != "no_safe_heading":
            last_failure = latest_reason
        if not ok:
            if sim.pose[:2] != start_xy:
                frame = agent.capture()
                continue
            scan_pose = sim.pose[:2]
            for frame in agent.scan():
                ok, next_reason = agent.evaluate(frame, tried)
                if ok:
                    break
                if next_reason != "no_safe_heading":
                    last_failure = next_reason
        if not ok:
            if sim.pose[:2] != scan_pose:
                frame = agent.capture()
                continue
            reason = (last_failure or "no_safe_heading") if agent.memory else "no_observed_targets"
            break
        tried.clear()
        last_failure = None
        # The simulator can end an episode when its objective is satisfied.
        # CoveragePolicy still decides solely from observations and feedback.
        if sim.all_collected:
            break
        frame = agent.capture()
    final = sim.final_state()
    if not final["remaining"]:
        reason = "all_collected"
    collected, remaining = final["collected"], final["remaining"]
    rotations, approach_m, pickup_m = 0., 0., 0.
    for left, right in zip(sim.trace, sim.trace[1:]):
        distance = math.dist(left["pose"][:2], right["pose"][:2])
        rotations += abs(normalize(right["pose"][2] - left["pose"][2]))
        if right["stage"] == "approach":
            approach_m += distance
        elif right["stage"] == "pickup":
            pickup_m += distance
    return {
        "case": name, "case_config": case, "seed": seed, "frame_id": "map", "scene_config": sim.world_config,
        "regions": asdict(sim.regions), "initial": sim.initial,
        "observations": sim.observations, "decisions": agent.decisions, "approaches": agent.approaches,
        "plans": agent.plans, "actions": sim.actions, "trace": sim.trace,
        "remaining": remaining, "collected": collected, "unresolved_observations": list(agent.memory.values()),
        "stop_reason": reason, "failure": final["failure"],
        "metrics": {"observations": len(sim.observations), "total_turn_rad": rotations,
                    "approach_distance_m": approach_m, "pickup_distance_m": pickup_m,
                    "pickup_stops": sum(len(action.get("pickups", ())) for action in sim.actions),
                    "elapsed_sim_s": sim.time, "completion_time_s": sim.time if not remaining else None},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="定向接近与分区拾取仿真")
    parser.add_argument("--case", default="two-clusters", help="场景名称，all 运行全部")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path, default=Path("sim/output/directional"))
    parser.add_argument("--show", action="store_true", help="打开 IR-SIM 窗口")
    parser.add_argument("--gif", action="store_true", help="离屏导出 IR-SIM GIF")
    parser.add_argument("--snapshot", action="store_true", help="导出拾取阶段的 IR-SIM 截图")
    parser.add_argument("--html", action="store_true", help="额外导出 HTML 回放")
    parser.add_argument("--deferred-bearings", action="store_true", help="启用二级方位线索复测")
    args = parser.parse_args(argv)
    try:
        cases = load_config(args.config)["custom"]["directional"]["cases"]
        if args.case not in cases and args.case != "all":
            parser.error(f"未知场景 {args.case}，可选 {', '.join(cases)}")
        for name in cases if args.case == "all" else (args.case,):
            policy = partial(CoveragePolicy, use_deferred_bearings=args.deferred_bearings)
            record = run_case(name, args.seed, args.config, policy=policy)
            output = args.output / f"{name}.json"
            write_json(output, record)
            print(f"{name}: {record['stop_reason']} · picked {len(record['collected'])} · {output}", flush=True)
            if args.html:
                from .direction_replay import write_replay

                write_replay(output.with_suffix(".html"), record)
            if args.show or args.gif or args.snapshot:
                from route_plan_sim.directional_viewer import show_replay

                show_replay(record, show=args.show,
                            gif=output.with_suffix(".gif") if args.gif else None,
                            snapshot=output.with_suffix(".png") if args.snapshot else None)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
