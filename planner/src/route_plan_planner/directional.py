"""Run and export a reproducible directional pickup episode."""

import argparse
from dataclasses import asdict, fields
from functools import partial
import math
from pathlib import Path

from route_plan_sim.directional import DirectionalScene, load_config
from route_plan_sim.scene import normalize

from .api import PlanningRequest
from .direction_policy import CoverageConfig, CoveragePolicy
from .records import write_json


def simulate_action(agent, sim, action):
    """Execute one strategy action using the simulator's continuous checks."""
    if action.kind == "observe":
        frame = sim.capture()
        agent.submit_observation(action.action_id, pose=sim.pose, time_s=sim.time,
                                 observation={**frame, "observation_id": frame["index"]})
        updated = agent.latest_frame
        sim.record_known(frame["index"], updated["known"], deferred=updated["deferred"],
                         clusters=updated.get("clusters"))
        return
    if action.kind == "turn":
        result = sim.turn(action.goals[0][2])
    elif action.kind == "forward":
        result = sim.forward(action.distance_m)
    elif action.kind == "pickup":
        stops = [asdict(stop) for stop in action.pickup_stops]
        positions = {target.id: (target.x, target.y) for target in action.targets}
        result = sim.execute_pickups(action.goals, stops, positions, frame_index=agent.latest_frame["index"],
                                     target_positions=positions)
    else:
        raise ValueError(f"Unexpected directional action {action.kind}")
    if agent.config.cluster_radius_m is not None and action.kind != "pickup":
        result["cluster_id"] = agent.active_cluster
        if action.kind == "turn" and agent.active_cluster is None:
            result["stage"] = "final_scan"
    picked_ids = [target.id for target in action.targets] if result["status"] == "success" else []
    agent.submit_result(action.action_id, status=result["status"], pose=sim.pose, time_s=sim.time,
                        picked_ids=picked_ids, reason=result["reason"])


def run_case(name, seed=42, config_path=None, *, policy=CoveragePolicy, cluster_radius_m=None):
    config = load_config(config_path)
    case = config["custom"]["directional"]["cases"][name]
    reference = config["custom"].get("robot_config")
    robot_config = (Path(config_path).resolve().parent / reference) if config_path and reference else None
    sim = DirectionalScene(config, name, seed, robot_config, planner_request=PlanningRequest.from_scene,
                           coarse_coordinates=cluster_radius_m is not None)
    regions = asdict(sim.regions)
    coverage = CoverageConfig(**{field.name: regions[field.name] for field in fields(CoverageConfig)
                                if field.name in regions},
                              probe_initial_heading=case.get("probe_initial_heading", False),
                              cluster_radius_m=cluster_radius_m)
    regions["cluster_radius_m"] = cluster_radius_m
    agent = policy(coverage, request_options=sim.request([]).to_dict())
    cluster_visits = []
    while cluster_radius_m is not None or not sim.all_collected:
        action = agent.next_action()
        while agent.is_next_cluster():
            cluster_visits.append(asdict(agent.next_cluster()))
        if action is None:
            break
        simulate_action(agent, sim, action)
    final = sim.final_state()
    completed = not final["remaining"]
    reason = "all_collected" if completed else agent.stop_reason
    for entry in agent.plans:
        entry["trace_index"] = sim.observations[entry["frame_index"]]["trace_index"]
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
        "regions": regions, "initial": sim.initial,
        "observations": sim.observations, "decisions": agent.decisions, "approaches": agent.approaches,
        "cluster_visits": cluster_visits,
        "plans": agent.plans, "actions": sim.actions, "trace": sim.trace,
        "remaining": remaining, "collected": collected, "unresolved_observations": list(agent.memory.values()),
        "stop_reason": reason, "policy_stop_reason": agent.stop_reason, "failure": final["failure"],
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
    parser.add_argument("--cluster-radius", type=float, metavar="METRES",
                        help="用该半径聚合粗测坐标，按簇搜索并在末尾环视")
    args = parser.parse_args(argv)
    if args.cluster_radius is not None and args.deferred_bearings:
        parser.error("--cluster-radius 与 --deferred-bearings 适用于不同的观测输入")
    try:
        cases = load_config(args.config)["custom"]["directional"]["cases"]
        if args.case not in cases and args.case != "all":
            parser.error(f"未知场景 {args.case}，可选 {', '.join(cases)}")
        for name in cases if args.case == "all" else (args.case,):
            policy = partial(CoveragePolicy, use_deferred_bearings=args.deferred_bearings)
            record = run_case(name, args.seed, args.config, policy=policy, cluster_radius_m=args.cluster_radius)
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
