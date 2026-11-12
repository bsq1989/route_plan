"""Example upper-layer loop driving the dual-camera IR-SIM cases."""

import argparse
from dataclasses import asdict
from importlib.resources import files
import json
import math
from pathlib import Path
from random import Random

import yaml

from .cameras import Camera, to_local, to_map
from .control import PoseController
from .scene import EPS, Scene, normalize


def load_demo(path=None):
    source = Path(path) if path is not None else files(__package__).joinpath("perception.yaml")
    return yaml.safe_load(source.read_text(encoding="utf-8"))


def _pose_radians(values):
    x, y, yaw_deg = map(float, values)
    return x, y, math.radians(yaw_deg)


def make_cameras(config):
    return {
        name: Camera(
            frame_id=values["frame_id"], mount=_pose_radians(values["mount_xy_yaw_deg"]),
            horizontal_fov=math.radians(values["horizontal_fov_deg"]),
            max_range=float(values["max_range_m"]), wide=name == "bottom",
            range_error_per_m=float(values.get("range_error_per_m", 0.)),
            merge_angle=math.radians(values.get("merge_angle_deg", 0.)),
        )
        for name, values in config["cameras"].items()
    }


def run_case(config, name, seed=42):
    from active_perception_sdk import ObservationCue, suggest_views

    case = config["cases"][name]
    cameras = make_cameras(config)
    scene = Scene(scenario="empty", seed=seed, obstacles=case.get("obstacles", ()))
    # Observation precedes pickup. Targets stay physical, even through a view change.
    scene.collection_mode = scene.config["custom"]["collection_mode"] = "at_stop"
    scene.pickup_rule = scene.config["custom"]["pickup_rule"] = "center"
    scene.add_targets(case["targets"])
    initial = scene.clone()
    motion = PoseController(scene)
    rng = Random(seed)
    dt = float(scene.config["world"]["step_time"])
    tick = 0
    trace = [{"timestamp_s": 0., "pose": list(scene.pose), "stage": "initial"}]
    observations, requests, actions = [], [], []

    def observe(stage):
        timestamp = round(tick * dt, 10)
        bottom, bottom_truth = cameras["bottom"].capture(scene, timestamp, rng)
        local, local_truth = cameras["local"].capture(scene, timestamp)
        observation = {
            "stage": stage, "trace_index": len(trace) - 1, "robot_pose_map": list(scene.pose),
            "bottom": bottom, "local": local,
            "truth_visibility": {"bottom": bottom_truth, "local": local_truth},
        }
        observations.append(observation)
        return observation

    def execute(goal, stage):
        nonlocal tick
        start = list(scene.pose)
        action = {
            "stage": stage, "goal_pose_map": list(goal), "start_pose": start,
            "start_timestamp_s": round(tick * dt, 10),
        }
        motion.set_goal(*goal)
        while motion.command is not None:
            motion.step(dt)
            tick += 1
            trace.append({"timestamp_s": round(tick * dt, 10), "pose": list(scene.pose), "stage": stage})
        action.update(end_pose=list(scene.pose), end_timestamp_s=round(tick * dt, 10),
                      failure=scene.failure)
        actions.append(action)

    def try_view(observation):
        wide, local = observation["bottom"], observation["local"]
        wide_pose, local_pose = wide["camera_pose_map"], local["camera_pose_map"]
        cues = [
            ObservationCue(
                normalize(wide_pose[2] + item["bearing_rad"] - local_pose[2]),
                to_local(local_pose, to_map(wide_pose, item["xy_m"])),
            )
            for item in wide["detections"]
        ]
        views = suggest_views(cues, horizontal_fov=cameras["local"].horizontal_fov)
        request = {
            "observation_index": len(observations) - 1, "timestamp_s": local["timestamp_s"],
            "frame_id": local["frame_id"], "cues": [asdict(cue) for cue in cues],
            "suggestions": [asdict(view) for view in views], "selected_index": None,
            "rejected": [],
        }
        requests.append(request)
        for index, view in enumerate(views):
            # A trial scene runs the same continuous footprint checks as execution.
            trial = scene.clone()
            trial.advance(rotation=view.yaw_offset)
            if trial.failure:
                request["rejected"].append({"index": index, "failure": trial.failure})
                continue
            request["selected_index"] = index
            execute((*scene.pose[:2], normalize(scene.pose[2] + view.yaw_offset)), "sdk_view")
            return observe("sdk_view") if not scene.failure else observation
        return observation

    observation = observe("initial")
    waypoints = iter(case.get("observation_poses_xy_yaw_deg", ()))
    while not observation["local"]["detections"] and not scene.failure:
        observation = try_view(observation)
        if observation["local"]["detections"] or scene.failure:
            break
        waypoint = next(waypoints, None)
        if waypoint is None:
            break
        execute(_pose_radians(waypoint), "preset_viewpoint")
        if not scene.failure:
            observation = observe("preset_viewpoint")

    local = observation["local"]
    handoff = None
    if local["detections"] and not scene.failure:
        handoff = {
            "timestamp_s": local["timestamp_s"], "frame_id": "map",
            "observation_index": len(observations) - 1, "source_frame_id": local["frame_id"],
            "robot_pose_map": list(scene.pose),
            "targets_xy_m": [list(to_map(local["camera_pose_map"], detection["xy_m"]))
                             for detection in local["detections"]],
        }
    return initial, {
        "case": name, "title": case["title"], "seed": seed, "map_frame_id": "map",
        "world_config": initial.config, "cameras": {name: asdict(camera) for name, camera in cameras.items()},
        "truth": {"initial_pose": list(initial.pose), "targets": initial.targets,
                  "target_diameter_m": initial.target_diameter, "obstacles_xy_radius_m": initial.obstacles},
        "observations": observations, "view_requests": requests, "actions": actions,
        "trace": trace, "handoff": handoff, "final": scene.snapshot(),
    }


def main():
    parser = argparse.ArgumentParser(description="运行双相机主动感知 IR-SIM 场景并导出回放")
    parser.add_argument("--config", type=Path, help="相机与场景 YAML")
    parser.add_argument("--case", help="只运行指定场景，默认运行全部")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=Path, default=Path("sim/output/active-perception"))
    args = parser.parse_args()
    try:
        import active_perception_sdk  # noqa: F401
    except ModuleNotFoundError as error:
        if error.name != "active_perception_sdk":
            raise
        parser.error("请安装主动感知 SDK 或使用 uv run --project sim --with-editable ./active_perception")
    config = load_demo(args.config)
    if args.case is not None and args.case not in config["cases"]:
        parser.error(f"未知场景 {args.case}，可选 {', '.join(config['cases'])}")

    from .perception_replay import save_replay

    args.output.mkdir(parents=True, exist_ok=True)
    failed = False
    for name in (args.case,) if args.case is not None else config["cases"]:
        initial, result = run_case(config, name, args.seed)
        record_path = args.output / f"{name}.json"
        record_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                               encoding="utf-8")
        save_replay(initial, make_cameras(config), result, args.output / f"{name}.gif")
        handoff = result["handoff"]
        failed |= handoff is None
        count = len(handoff["targets_xy_m"]) if handoff else 0
        print(f"{name} 交接 {count} 个目标 · {record_path}")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
