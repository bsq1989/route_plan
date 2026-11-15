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
    body = {"length": scene.dimensions["body_length"], "width": scene.dimensions["body_width"],
            "pose_reference": scene.body_pose_reference, "rotation_center": scene.body_pose_reference}
    if scene.body_pose_reference == "base_link":
        body["front_x"] = scene.body.xmax
    if "body_height" in custom:
        body["height"] = custom["body_height"]
    return {
        "world_config": deepcopy(scene.config),
        "vehicle_config": {
            "version": 1, "units": "m",
            "body": body,
            "collector": deepcopy(custom["collector"]),
            "safety": {"body_margin": scene.body_margin},
            "motion": {"forward_speed": custom["forward_speed"],
                       "angular_speed": custom["angular_speed"]},
        },
        "state": scene.snapshot(),
    }


def restore_scene(record: dict, *, allow_safety_intrusions: bool = False) -> Scene:
    config = deepcopy(record["world_config"])
    state = record["state"]
    if state.get("failure"):
        raise ValueError("Cannot replay a failed initial scene")
    config["robot"]["state"] = state["pose"]
    config["custom"]["random_count"] = 0
    config["custom"]["rule_version"] = config["custom"].get("rule_version", 1)
    # SDK <=0.2.0 stop archives accepted disk contact. Replay them honestly;
    # newly generated stop requests explicitly record the centre rule.
    config["custom"]["pickup_rule"] = config["custom"].get("pickup_rule", "contact")
    # Older records used mathematical points. Preserve that model explicitly.
    diameter = state.get("target_diameter", config["custom"].get("target_diameter", 0.))
    if "target_diameter" in state and "target_diameter" in config["custom"]:
        if state["target_diameter"] != config["custom"]["target_diameter"]:
            raise ValueError("Target diameter differs between state and configuration")
    config["custom"]["target_diameter"] = diameter
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
        remaining[key] = tuple(point)
    keys = scene.add_targets(remaining.values(), allow_safety_intrusions=allow_safety_intrusions)
    if scene.collected:
        raise ValueError("Remaining target group is already inside collector")
    if scene.recovering:
        scene._initial_intrusions = frozenset(archived for current, archived in zip(keys, remaining)
                                              if current in scene._initial_intrusions)
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
    collector = scene.config["custom"]["collector"]
    height = scene.config["custom"].get("body_height")
    scene.config = deepcopy(record["world_config"])
    scene.config["custom"]["collector"] = collector
    scene.config["custom"]["body_pose_reference"] = scene.body_pose_reference
    if scene.body_pose_reference == "base_link":
        scene.config["custom"]["body_front_x"] = scene.body.xmax
    else:
        scene.config["custom"].pop("body_front_x", None)
    if height is None:
        scene.config["custom"].pop("body_height", None)
    else:
        scene.config["custom"]["body_height"] = height
    scene.config["custom"]["target_diameter"] = diameter
    scene.config["custom"]["rule_version"] = scene.rule_version
    scene.config["custom"]["pickup_rule"] = scene.pickup_rule
    return scene


def write_json(path: Path, payload: dict):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def read_json(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))
