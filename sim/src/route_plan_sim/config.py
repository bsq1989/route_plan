"""Resolve one vehicle profile independently of the map and process cwd."""

from copy import deepcopy
from importlib.resources import files
import math
from pathlib import Path

import yaml


def default_robot_config() -> Path:
    checkout = Path(__file__).resolve().parents[2] / "config" / "robot.yaml"
    if checkout.is_file():
        return checkout
    # The wheel includes the same source file, so installed usage also works.
    return Path(str(files(__package__).joinpath("config/robot.yaml")))


def load_config(world_path: Path, robot_path: str | Path | None = None) -> tuple[dict, Path]:
    world = yaml.safe_load(world_path.read_text(encoding="utf-8"))
    if not isinstance(world, dict):
        raise ValueError("场景配置必须是 YAML 映射")
    reference = world.get("custom", {}).get("robot_config")
    if robot_path is not None:
        path = Path(robot_path).expanduser().resolve()
    elif reference:
        path = (world_path.parent / reference).resolve()
    else:
        path = default_robot_config().resolve()
    # Do not silently substitute demo dimensions if a file is missing/invalid.
    profile = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(profile, dict) or profile.get("version") != 1:
        raise ValueError("车辆配置必须是 YAML 映射且 version 为 1")
    if profile.get("units") != "m":
        raise ValueError("车辆配置 units 必须为 m，尺寸请先换算成米")
    try:
        body = profile["body"]
        if body["pose_reference"] != "body_center" or body["rotation_center"] != "body_center":
            raise ValueError("当前模型仅支持车身中心作为 pose 参考点和旋转中心")
        raw = {"body_length": body["length"], "body_width": body["width"],
               "collector_width": profile["collector"]["width"],
               "collector_depth": profile["collector"]["depth"],
               "collector_gap": profile["collector"].get("gap", 0),
               "collector_offset": profile["collector"].get("lateral_offset", 0),
               "body_margin": profile["safety"]["body_margin"],
               "forward_speed": profile["motion"]["forward_speed"],
               "angular_speed": profile["motion"]["angular_speed"]}
        values = {key: float(value) for key, value in raw.items()}
    except (KeyError, TypeError) as error:
        raise ValueError(f"车辆配置缺少字段或结构错误：{error}") from error
    for key, value in values.items():
        if not math.isfinite(value):
            raise ValueError(f"车辆配置 {key} 必须为有限数值")
        if key in ("body_margin", "collector_gap"):
            if value < 0:
                raise ValueError(f"车辆配置 {key} 不能小于 0")
        elif key != "collector_offset" and value <= 0:
            raise ValueError(f"车辆配置 {key} 必须大于 0")
    merged = deepcopy(world)
    robot = merged.setdefault("robot", {})
    robot["kinematics"] = {"name": "diff"}
    robot["shape"] = {"name": "rectangle", "length": values["body_length"], "width": values["body_width"]}
    custom = merged.setdefault("custom", {})
    custom["collector"] = {"width": values["collector_width"], "depth": values["collector_depth"],
                           "gap": values["collector_gap"], "lateral_offset": values["collector_offset"]}
    for key in ("body_margin", "forward_speed", "angular_speed"):
        custom[key] = values[key]
    return merged, path
