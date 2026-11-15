"""Resolve one vehicle profile independently of the map and process cwd."""

from copy import deepcopy
from importlib.resources import files
import math
from pathlib import Path

import yaml
from .boundary import PolygonFootprint


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
        collector = profile["collector"]
        reference = body["pose_reference"]
        if reference not in ("body_center", "base_link") or body["rotation_center"] != reference:
            raise ValueError("车体位姿参考点须与旋转中心一致")
        body_polygon = PolygonFootprint.from_points(body["polygon"]) if "polygon" in body else None
        if body_polygon is not None:
            body_polygon.outward_offset(0.)
            if not body_polygon.polygon.contains_disk(0., 0.):
                raise ValueError("Body polygon must contain the pose reference in its interior")
        polygon = PolygonFootprint.from_points(collector["polygon"]) if "polygon" in collector else None
        length = body["length"] if body_polygon is None else body_polygon.xmax - body_polygon.xmin
        width = body["width"] if body_polygon is None else body_polygon.ymax - body_polygon.ymin
        raw = {"body_length": length, "body_width": width,
               "body_margin": profile["safety"]["body_margin"],
               "forward_speed": profile["motion"]["forward_speed"],
               "angular_speed": profile["motion"]["angular_speed"]}
        if reference == "base_link":
            raw["body_front_x"] = body["front_x"] if body_polygon is None else body_polygon.xmax
        if polygon is None:
            raw.update(collector_width=collector["width"], collector_depth=collector["depth"],
                       collector_offset=collector.get("lateral_offset", 0))
            if "gap" in collector:
                raw["collector_gap"] = collector["gap"]
        if "height" in body:
            raw["body_height"] = body["height"]
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
    custom["body_pose_reference"] = reference
    if body_polygon is None:
        custom.pop("body_polygon", None)
    else:
        custom["body_polygon"] = body_polygon.corners
    if reference == "base_link":
        custom["body_front_x"] = values["body_front_x"]
    else:
        custom.pop("body_front_x", None)
    if "body_height" in values:
        custom["body_height"] = values["body_height"]
    else:
        custom.pop("body_height", None)
    if polygon is not None:
        custom["collector"] = {"polygon": polygon.corners}
    else:
        custom["collector"] = {"width": values["collector_width"], "depth": values["collector_depth"],
                               "lateral_offset": values["collector_offset"]}
        if "collector_gap" in values:
            custom["collector"]["gap"] = values["collector_gap"]
    for key in ("body_margin", "forward_speed", "angular_speed"):
        custom[key] = values[key]
    return merged, path
