"""Run and export a reproducible directional pickup episode."""

import argparse
from dataclasses import asdict
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
        frame = agent.approach(frame["detections"][0]["id"], frame)
        agent.pickup(frame)
        frame = agent.capture()
    reason, tried = "round_limit", set()
    last_failure = None
    for _ in range(sim.regions.maximum_rounds):
        start_xy = sim.pose[:2]
        ok, latest_reason = agent.evaluate(frame, tried)
        if not ok and latest_reason != "no_safe_heading":
            last_failure = latest_reason
        if not ok:
            scan_pose = sim.pose[:2]
            for frame in agent.scan():
                ok, next_reason = agent.evaluate(frame, tried)
                if ok:
                    break
                if next_reason != "no_safe_heading":
                    last_failure = next_reason
        if not ok:
            if sim.pose[:2] != scan_pose or sim.pose[:2] != start_xy:
                frame = agent.capture()
                continue
            reason = (last_failure or "no_safe_heading") if agent.memory else "no_observed_targets"
            break
        tried.clear()
        last_failure = None
        agent.frames_seen.clear()
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
    args = parser.parse_args(argv)
    try:
        cases = load_config(args.config)["custom"]["directional"]["cases"]
        if args.case not in cases and args.case != "all":
            parser.error(f"未知场景 {args.case}，可选 {', '.join(cases)}")
        from .direction_replay import write_replay

        for name in cases if args.case == "all" else (args.case,):
            record = run_case(name, args.seed, args.config)
            write_json(args.output / f"{name}.json", record)
            write_replay(args.output / f"{name}.html", record)
            print(f"{name}: {record['stop_reason']} · picked {len(record['collected'])} · {args.output / (name + '.html')}")
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
