"""Self-contained scene/plan records; no live config file is needed for replay."""

from copy import deepcopy
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml

from route_plan_sim import Scene


def describe_scene(scene: Scene) -> dict:
    custom = scene.config["custom"]
    return {
        "world_config": deepcopy(scene.config),
        "vehicle_config": {
            "version": 1, "units": "m",
            "body": {"length": scene.dimensions["body_length"],
                     "width": scene.dimensions["body_width"],
                     "pose_reference": "body_center", "rotation_center": "body_center"},
            "collector": dict(custom["collector"]),
            "safety": {"body_margin": scene.body_margin},
            "motion": {"forward_speed": custom["forward_speed"],
                       "angular_speed": custom["angular_speed"]},
        },
        "state": scene.snapshot(),
    }


def restore_scene(record: dict) -> Scene:
    config = deepcopy(record["world_config"])
    state = record["state"]
    if state.get("failure"):
        raise ValueError("Cannot replay a failed initial scene")
    config["robot"]["state"] = state["pose"]
    config["custom"]["random_count"] = 0
    # Use normal constructor/config validation, including the stored geometry.
    with TemporaryDirectory(prefix="route-plan-replay-") as directory:
        world = Path(directory) / "world.yaml"
        vehicle = Path(directory) / "robot.yaml"
        world.write_text(yaml.safe_dump(config), encoding="utf-8")
        vehicle.write_text(yaml.safe_dump(record["vehicle_config"]), encoding="utf-8")
        scene = Scene(config=world, scenario="random", seed=state["seed"], robot_config=vehicle)
    remaining = {}
    for raw_key, point in state["remaining"].items():
        key = int(raw_key)
        before = len(scene.targets)
        scene.add_target(*point)
        if len(scene.targets) != before + 1:
            raise ValueError("Remaining target is duplicated or already inside collector")
        remaining[key] = tuple(point)
    scene.targets = remaining
    scene.collected = {int(key): tuple(point) for key, point in state["collected"].items()}
    for point in scene.collected.values():
        if len(point) != 2 or not all(math.isfinite(x) for x in point):
            raise ValueError("Invalid collected target")
    if scene.targets.keys() & scene.collected.keys():
        raise ValueError("Remaining and collected target IDs overlap")
    scene._next_id = max([0, *scene.targets, *scene.collected]) + 1
    scene.scenario = state["scenario"]
    # Retain the stored source configuration for subsequent record exports.
    scene.config = deepcopy(record["world_config"])
    return scene


def write_json(path: Path, payload: dict):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def read_json(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))
