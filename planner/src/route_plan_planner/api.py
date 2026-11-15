"""Public, headless request/response API, independent of viewer configuration."""

from copy import deepcopy
from dataclasses import asdict, dataclass, field, fields
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import time
from typing import Mapping

import yaml

from route_plan_sim import Scene
from route_plan_sim.boundary import Fence
from route_plan_sim.config import default_robot_config
from route_plan_sim.scene import normalize

from .core import PlannerOptions, StaticPlanner
from .length import LengthPlanner
from .stops import StopsPlanner
from .motion import execute
from .records import describe_scene

TargetId = str | int
Pose = tuple[float, float, float]
DEFAULT_AUTO_FENCE_RADIUS_M = 1.4


def _number(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


@dataclass(frozen=True)
class Target:
    id: TargetId
    x: float
    y: float

    def __post_init__(self):
        if (isinstance(self.id, bool) or not isinstance(self.id, (str, int))
                or isinstance(self.id, str) and not self.id.strip()):
            raise ValueError("Target id must be an integer or a nonempty string")
        object.__setattr__(self, "x", _number(self.x, "target.x"))
        object.__setattr__(self, "y", _number(self.y, "target.y"))


@dataclass(frozen=True)
class PlanningRequest:
    width: float | None = None
    height: float | None = None
    pose: Pose | None = None
    targets: tuple[Target, ...] | None = None
    target_diameter: float = .10
    method: str = "length"
    timeout: float = 3.
    use_ompl: bool = True
    seed: int = 42
    robot_config: dict | str | Path | None = None
    frame_id: str = "map"
    request_id: str | None = None
    schema_version: int = 1
    fence: tuple[tuple[float, float], ...] | None = None
    fence_enabled: bool = True
    map_offset: tuple[float, float] = (0., 0.)
    mode: str = "standard"
    auto_fence_radius_m: float = DEFAULT_AUTO_FENCE_RADIUS_M

    def __post_init__(self):
        if self.mode not in ("standard", "auto_fence"):
            raise ValueError("mode must be standard or auto_fence")
        radius = _number(self.auto_fence_radius_m, "auto_fence_radius_m")
        if radius <= 0.:
            raise ValueError("auto_fence_radius_m must be positive")
        object.__setattr__(self, "auto_fence_radius_m", radius)
        if not isinstance(self.fence_enabled, bool):
            raise ValueError("fence_enabled must be boolean")
        fence_span = None
        if self.fence_enabled and self.fence is not None:
            fence = Fence.from_points(self.fence)
            xmin, ymin, xmax, ymax = fence.bounds
            fence_span = xmax - xmin, ymax - ymin
            object.__setattr__(self, "fence", fence.vertices)
            if self.width is None:
                object.__setattr__(self, "width", fence_span[0])
            if self.height is None:
                object.__setattr__(self, "height", fence_span[1])
        try:
            offset = tuple(_number(value, "map_offset") for value in self.map_offset)
        except TypeError as error:
            raise ValueError("map_offset must contain x and y in metres") from error
        if len(offset) != 2:
            raise ValueError("map_offset must contain x and y in metres")
        object.__setattr__(self, "map_offset", offset)
        temporary_map = (not self.fence_enabled or self.mode == "auto_fence") and (
            self.width is None and self.height is None)
        dimensions = () if temporary_map else ("width", "height")
        for name in (*dimensions, "timeout", "target_diameter"):
            value = _number(getattr(self, name), name)
            if value < 0 or value == 0 and name != "target_diameter":
                raise ValueError(f"{name} must be {'nonnegative' if name == 'target_diameter' else 'positive'}")
            object.__setattr__(self, name, value)
        if not temporary_map and self.target_diameter >= min(fence_span or (self.width, self.height)):
            raise ValueError("Target diameter must be smaller than the map")
        try:
            pose = tuple(_number(value, "pose") for value in self.pose)
        except TypeError as error:
            raise ValueError("pose must contain x, y, yaw in radians") from error
        if len(pose) != 3:
            raise ValueError("pose must contain x, y, yaw in radians")
        object.__setattr__(self, "pose", (pose[0], pose[1], normalize(pose[2])))
        if self.method not in ("feasibility", "length", "stops"):
            raise ValueError("method must be feasibility, length or stops")
        if not isinstance(self.use_ompl, bool):
            raise ValueError("use_ompl must be boolean")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or not 0 <= self.seed < 2**32:
            raise ValueError("seed must be an integer in [0, 2**32)")
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("Unsupported request schema_version")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("frame_id must be a nonempty string")
        if self.request_id is not None and not isinstance(self.request_id, str):
            raise ValueError("request_id must be a string or None")
        try:
            targets = tuple(item if isinstance(item, Target) else Target(**item) for item in self.targets)
        except (TypeError, KeyError) as error:
            raise ValueError("targets must contain Target objects or {id, x, y} mappings") from error
        if len({item.id for item in targets}) != len(targets):
            raise ValueError("Target ids must be unique")
        object.__setattr__(self, "targets", targets)
        if isinstance(self.robot_config, Mapping):
            object.__setattr__(self, "robot_config", deepcopy(dict(self.robot_config)))
        elif self.robot_config is not None and not isinstance(self.robot_config, (str, Path)):
            raise ValueError("robot_config must be a profile mapping, YAML path, or None")

    @classmethod
    def from_scene(cls, scene: Scene, **options):
        """Use a static scene's pose, targets, vehicle and map configuration."""
        record = describe_scene(scene)
        world, custom = scene.config["world"], scene.config["custom"]
        return cls.from_dict({
            "width": world["width"], "height": world["height"], "map_offset": world.get("offset", (0., 0.)),
            "pose": scene.pose, "targets": [{"id": key, "x": x, "y": y} for key, (x, y) in scene.targets.items()],
            "target_diameter": scene.target_diameter, "robot_config": record["vehicle_config"],
            "fence": custom.get("fence"), "fence_enabled": scene.fence_enabled, "seed": scene.seed,
            **options,
        })

    @classmethod
    def from_dict(cls, payload: Mapping):
        if not isinstance(payload, Mapping):
            raise ValueError("Request must be a mapping")
        unknown = set(payload) - {item.name for item in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown request fields: {sorted(map(str, unknown))}")
        try:
            return cls(**deepcopy(dict(payload)))
        except TypeError as error:
            raise ValueError(f"Invalid request: {error}") from error

    def to_dict(self):
        result = asdict(self)
        result["pose"] = list(self.pose)
        result["targets"] = [asdict(target) for target in self.targets]
        if self.fence is None:
            del result["fence"]
        elif self.fence_enabled:
            result["fence"] = [list(point) for point in self.fence]
        if self.fence_enabled:
            del result["fence_enabled"]
        if self.map_offset == (0., 0.):
            del result["map_offset"]
        else:
            result["map_offset"] = list(self.map_offset)
        if self.mode == "standard":
            del result["mode"]
            if self.auto_fence_radius_m == DEFAULT_AUTO_FENCE_RADIUS_M:
                del result["auto_fence_radius_m"]
        if isinstance(self.robot_config, Path):
            result["robot_config"] = str(self.robot_config)
        return result


@dataclass(frozen=True)
class PlanStep:
    goal: Pose
    expected_collected_ids: tuple[TargetId, ...] = ()


@dataclass(frozen=True)
class PickupStop:
    goal_index: int
    pose: Pose
    target_ids: tuple[TargetId, ...]


@dataclass
class PlanningResponse:
    status: str
    frame_id: str
    request_id: str | None
    goals: list[Pose] = field(default_factory=list)
    steps: list[PlanStep] = field(default_factory=list)
    length: float | None = None
    elapsed: float = 0.
    final_pose: Pose | None = None
    initially_collected_ids: list[TargetId] = field(default_factory=list)
    expected_collected_ids: list[TargetId] = field(default_factory=list)
    remaining_ids: list[TargetId] = field(default_factory=list)
    reason: str | None = None
    optimization: dict | None = None
    metrics: dict = field(default_factory=dict)
    resolved_request: dict = field(default_factory=dict)
    schema_version: int = 1
    pickup_stops: list[PickupStop] = field(default_factory=list)
    stop_count: int | None = None

    @property
    def success(self):
        return self.status == "success"

    def to_dict(self):
        # Return JSON-native containers and a snapshot rather than shared lists.
        result = asdict(self)
        result["success"] = self.success
        result["goals"] = [list(goal) for goal in self.goals]
        result["steps"] = [{"goal": list(step.goal), "expected_collected_ids": list(step.expected_collected_ids)}
                           for step in self.steps]
        result["final_pose"] = list(self.final_pose) if self.final_pose is not None else None
        result["pickup_stops"] = [{"goal_index": stop.goal_index, "pose": list(stop.pose),
                                    "target_ids": list(stop.target_ids)} for stop in self.pickup_stops]
        return result


def _robot_profile(profile):
    if not isinstance(profile, Mapping):
        path = Path(profile).expanduser() if profile is not None else default_robot_config()
        try:
            profile = yaml.safe_load(path.read_text(encoding="utf-8"))
        except yaml.YAMLError as error:
            raise ValueError(f"Invalid robot_config YAML: {error}") from error
    return deepcopy(profile)


def _empty_scene(request, *, extra_padding=0.):
    profile = _robot_profile(request.robot_config)
    world = {"world": {"width": request.width, "height": request.height, "step_time": .04,
                       "offset": list(request.map_offset)},
             "robot": {"state": list(request.pose)},
             "custom": {"random_count": 0, "target_diameter": request.target_diameter,
                        "pickup_rule": "center" if request.method == "stops" else "contact",
                        "collection_mode": "at_stop" if request.method == "stops" else "contact"}}
    if not request.fence_enabled:
        world["custom"]["fence_enabled"] = False
    if request.fence is not None:
        world["custom"]["fence"] = deepcopy(request.fence)
    if request.mode == "auto_fence":
        world["custom"]["motion_circle"] = [*request.pose[:2], request.auto_fence_radius_m]
    if request.width is None:
        # Bound the search around the observations in their original frame.
        # Padding covers the full vehicle and an optional local motion limit.
        radius = request.auto_fence_radius_m if request.mode == "auto_fence" else 0.
        padding = _workspace_padding(profile, request.target_diameter, max(extra_padding, radius))
        xs = [request.pose[0], *(target.x for target in request.targets)]
        ys = [request.pose[1], *(target.y for target in request.targets)]
        world["world"].update(width=max(xs) - min(xs) + 2 * padding,
                              height=max(ys) - min(ys) + 2 * padding,
                              offset=[min(xs) - padding, min(ys) - padding])
    with TemporaryDirectory(prefix="route-plan-input-") as directory:
        world_path, robot_path = Path(directory) / "world.yaml", Path(directory) / "robot.yaml"
        world_path.write_text(yaml.safe_dump(world), encoding="utf-8")
        try:
            robot_path.write_text(yaml.safe_dump(profile), encoding="utf-8")
        except yaml.YAMLError as error:
            raise ValueError(f"Invalid robot_config: {error}") from error
        return Scene(config=world_path, robot_config=robot_path, scenario="random", seed=request.seed)


def _workspace_padding(profile, diameter, extra):
    try:
        body, collector = profile["body"], profile["collector"]
        margin = profile["safety"]["body_margin"]
        extents = (body["length"], body["width"], margin)
        if "polygon" in collector:
            extents += (max(math.hypot(*map(float, point)) for point in collector["polygon"]),)
        else:
            extents += (collector["width"], collector["depth"],
                        collector.get("gap", margin), collector.get("lateral_offset", 0.))
    except (AttributeError, KeyError, TypeError) as error:
        raise ValueError("Invalid robot_config geometry") from error
    return 1. + extra + diameter + sum(abs(_number(value, "robot dimension")) for value in extents)


def _scene(request, *, allow_safety_intrusions=False, extra_padding=0.):
    scene = _empty_scene(request, extra_padding=extra_padding)
    points = ((target.x, target.y) for target in request.targets)
    keys = scene.add_targets(points, allow_safety_intrusions=allow_safety_intrusions)
    id_map = {key: target.id for key, target in zip(keys, request.targets)}
    return scene, id_map, describe_scene(scene)["vehicle_config"]


def plan(request: PlanningRequest | Mapping) -> PlanningResponse:
    """Plan a complete static task. This function sends no hardware commands.

    Invalid input/configuration raises ValueError/OSError. Search failures return
    a response without executable goals. Each call owns a separate planner.
    """
    started = time.monotonic()
    request = request if isinstance(request, PlanningRequest) else PlanningRequest.from_dict(request)
    scene, id_map, profile = _scene(request, allow_safety_intrusions=request.method == "stops")
    initially_collected = [id_map[key] for key in scene.collected]
    options = PlannerOptions(timeout=request.timeout, ompl=request.use_ompl, seed=request.seed)
    planner = {"length": LengthPlanner, "feasibility": StaticPlanner, "stops": StopsPlanner}[request.method]
    result = planner(options).plan(scene)
    resolved = request.to_dict()
    resolved["robot_config"] = profile
    response = PlanningResponse(status=result.status, frame_id=request.frame_id, request_id=request.request_id,
                                reason=result.reason, remaining_ids=[id_map[key] for key in scene.targets],
                                initially_collected_ids=initially_collected, resolved_request=resolved,
                                expected_collected_ids=list(initially_collected),
                                optimization=deepcopy(result.optimization),
                                metrics={"pickup_rule": scene.pickup_rule, "collection_mode": scene.collection_mode,
                                         "expanded": result.expanded, "trials": result.trials,
                                         "backtracks": result.backtracks, "ompl_calls": result.ompl_calls,
                                         "best_remaining": result.best_remaining,
                                         "planner_elapsed": result.elapsed})
    escape_count = (result.optimization or {}).get("escape_goal_count", 0)
    if escape_count:
        response.metrics["escape_goal_count"] = escape_count
    if result.status == "success":
        trace = scene.clone()
        steps = []
        try:
            pickup_stops = {stop["goal_index"]: stop for stop in result.pickup_stops}
            for index, goal in enumerate(result.goals):
                before = set(trace.collected)
                execute(trace, [goal], minimum_motion=request.method == "stops")
                if index in pickup_stops:
                    removed = trace.collect_at_stop()
                    if set(removed) != set(pickup_stops[index]["target_ids"]):
                        raise ValueError("Pickup stop coverage differs from the plan")
                steps.append(PlanStep(tuple(goal), tuple(id_map[key] for key in trace.collected if key not in before)))
        except ValueError as error:
            response.status, response.reason = "replay_failed", str(error)
        else:
            if trace.failure or trace.targets:
                response.status, response.reason = "replay_failed", trace.failure or "Incomplete final plan"
            else:
                response.goals = [tuple(goal) for goal in result.goals]
                response.steps = steps
                response.length = result.length
                response.final_pose = tuple(trace.pose)
                response.expected_collected_ids = [id_map[key] for key in trace.collected]
                response.remaining_ids = []
                response.pickup_stops = [PickupStop(stop["goal_index"], tuple(stop["pose"]),
                                                    tuple(id_map[key] for key in stop["target_ids"]))
                                         for stop in result.pickup_stops]
                response.stop_count = len(response.pickup_stops) if request.method == "stops" else None
    response.elapsed = time.monotonic() - started
    return response
