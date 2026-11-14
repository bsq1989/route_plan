"""Stopped observations and atomic, full-truth motion checks."""

from copy import deepcopy
from dataclasses import asdict, dataclass
from importlib.resources import files
import math
from pathlib import Path
from random import Random
from tempfile import TemporaryDirectory

import yaml

from .control import PoseController
from .scene import EPS, Scene, normalize

MIN_DISTANCE = .05
MIN_TURN = math.radians(1)


@dataclass(frozen=True)
class Regions:
    view_range_m: float
    view_angle_deg: float
    pickup_inner_radius_m: float
    pickup_outer_radius_m: float
    pickup_angle_deg: float
    approach_tolerance_m: float
    precision_range_m: float
    auto_fence_radius_m: float
    coarse_radial_error: float
    miss_probability: float
    maximum_rounds: int

    def __post_init__(self):
        if not all(math.isfinite(value) for value in asdict(self).values()):
            raise ValueError("Directional parameters must be finite")
        if (not 0 < self.view_angle_deg <= 360 or not 0 < self.pickup_angle_deg <= 360
                or not 0 < self.pickup_inner_radius_m < self.pickup_outer_radius_m
                or min(self.view_range_m, self.precision_range_m, self.auto_fence_radius_m) <= 0
                or not 0 <= self.approach_tolerance_m <= self.pickup_outer_radius_m - self.pickup_inner_radius_m
                or not 0 <= self.coarse_radial_error < 1 or not 0 <= self.miss_probability <= 1
                or type(self.maximum_rounds) is not int or self.maximum_rounds < 1):
            raise ValueError("Invalid directional regions")

    def eligible(self, pose, point):
        radius, bearing = relative(pose, point)
        return (self.pickup_inner_radius_m - EPS <= radius <= self.pickup_outer_radius_m + EPS
                and abs(bearing) <= math.radians(self.pickup_angle_deg / 2) + EPS)


def relative(pose, point):
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    return math.hypot(dx, dy), normalize(math.atan2(dy, dx) - pose[2])


def load_config(path=None):
    source = Path(path) if path else files(__package__).joinpath("directional.yaml")
    return yaml.safe_load(source.read_text(encoding="utf-8"))


class DirectionalScene:
    def __init__(self, config, name, seed, robot_config=None, *, planner_request):
        options = config["custom"]["directional"]
        case = options["cases"][name]
        params = {key: case.get(key, value) for key, value in options.items() if key != "cases"}
        self.regions = Regions(**params)
        world = {"world": config["world"], "robot": {**config["robot"]},
                 "custom": {key: value for key, value in config["custom"].items() if key != "directional"}}
        world["custom"].update(collection_mode="at_stop", pickup_rule="center")
        world["robot"]["state"] = case.get("pose", world["robot"]["state"])
        if robot_config is None and world["custom"].get("robot_config"):
            raise ValueError("Pass the resolved robot config path")
        with TemporaryDirectory(prefix="directional-") as directory:
            path = Path(directory) / "world.yaml"
            path.write_text(yaml.safe_dump(world), encoding="utf-8")
            self._scene = Scene(path, scenario="empty", seed=seed, robot_config=robot_config,
                                obstacles=case.get("obstacles"))
        self._scene.add_targets(case["targets"])
        self._planner_request = planner_request
        self._noise = Random(seed)
        self.observations, self.actions, self.trace = [], [], []
        self.initial = self._scene.snapshot()
        self.world_config = deepcopy(self._scene.config)
        self.time = 0.
        self.trace.append({"time_s": 0., "pose": self.pose, "stage": "initial",
                           "auto_fence": self._scene.motion_circle, "collected_ids": []})

    @property
    def pose(self):
        return self._scene.pose

    def capture(self):
        scene, params = self._scene, self.regions
        detections = []
        for key, point in scene.targets.items():
            distance, angle = relative(scene.pose, point)
            if distance > params.view_range_m + EPS or abs(angle) > math.radians(params.view_angle_deg / 2) + EPS:
                continue
            blockers = [*scene.obstacles, *((*other, scene.target_radius) for k, other in scene.targets.items() if k != key)]
            if self._occluded(distance, angle + scene.pose[2], blockers):
                continue
            if params.miss_probability and self._noise.random() < params.miss_probability:
                continue
            precise = distance <= params.precision_range_m + EPS
            xy = point
            if not precise:
                factor = 1 + self._noise.uniform(-params.coarse_radial_error, params.coarse_radial_error)
                xy = tuple(origin + (target - origin) * factor for origin, target in zip(scene.pose, point))
            detections.append({"id": key, "xy_m": xy, "quality": "precise" if precise else "coarse"})
        frame = {"index": len(self.observations), "time_s": self.time, "pose": scene.pose, "frame_id": "map",
                 "view_range_m": params.view_range_m, "view_angle_deg": params.view_angle_deg,
                 "trace_index": len(self.trace) - 1, "detections": detections}
        self.observations.append(deepcopy(frame))
        return frame

    def record_known(self, index, detections):
        self.observations[index]["known"] = [dict(item) for item in detections]

    def _occluded(self, distance, yaw, blockers):
        x, y = self.pose[:2]
        c, s = math.cos(yaw), math.sin(yaw)
        length = max(0., distance - self._scene.target_radius)
        for bx, by, radius in blockers:
            dx, dy = bx - x, by - y
            projected = max(0., min(length, dx * c + dy * s))
            if math.hypot(dx - projected * c, dy - projected * s) <= radius + EPS:
                return True
        return False

    def turn(self, yaw):
        return self._run("turn", [(*self.pose[:2], yaw)])

    def forward(self, distance):
        x, y, yaw = self.pose
        if not math.isfinite(distance) or distance <= MIN_DISTANCE + EPS:
            return self._reject("approach", "translation_deadband")
        return self._run("approach", [(x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)])

    def request(self, targets):
        return self._planner_request(
            self._scene, method="stops", mode="auto_fence", use_ompl=False,
            auto_fence_radius_m=self.regions.auto_fence_radius_m, targets=targets, timeout=2.,
        )

    def execute_pickups(self, goals, stops, allowed_ids):
        return self._run("pickup", goals, stops, allowed_ids)

    def _reject(self, stage, reason, goals=()):
        result = {"stage": stage, "status": "blocked", "reason": reason, "start_pose": self.pose,
                  "end_pose": self.pose, "goals": goals,
                  "trace_start": len(self.trace) - 1, "trace_end": len(self.trace) - 1}
        self.actions.append(result)
        return result

    def _run(self, stage, goals, stops=(), allowed_ids=()):
        # Stage mutations on a clone. Commit the complete verified trajectory in
        # one operation, so a later stop conflict cannot partially delete targets.
        trial = self._scene.clone()
        original_circle = trial.motion_circle
        circle = (*self.pose[:2], self.regions.auto_fence_radius_m) if stage == "pickup" else original_circle
        trial.motion_circle = circle
        controller = PoseController(trial, minimum_translation=MIN_DISTANCE, minimum_rotation=MIN_TURN)
        start_pose, start_index = self.pose, len(self.trace) - 1
        schedule = {stop["goal_index"]: set(stop["target_ids"]) for stop in stops}
        allowed_ids = set(allowed_ids)
        collected = set()
        frames, pickups = [], []
        if (len(schedule) != len(stops) or any(type(i) is not int or not 0 <= i < len(goals) for i in schedule)
                or any(not ids or not ids <= allowed_ids for ids in schedule.values())):
            return self._reject(stage, "invalid_pickup_stops", goals)
        dt = float(trial.config["world"]["step_time"])
        custom = trial.config["custom"]
        try:
            for index, goal in enumerate(goals):
                if not trial.contains_motion_point(*goal[:2]):
                    return self._reject(stage, "Motion circle boundary", goals)
                controller.set_goal(*goal)
                while controller.command is not None:
                    kind, remaining = controller.command
                    speed = custom["forward_speed" if kind == "move" else "angular_speed"]
                    elapsed = min(dt, abs(remaining) / speed)
                    controller.step(elapsed)
                    if trial.failure:
                        return self._reject(stage, trial.failure, goals)
                    frames.append((trial.pose, list(trial.collected), elapsed, stage))
                if index in schedule:
                    removed = set(trial.collect_at_stop())
                    if removed != schedule[index]:
                        return self._reject(stage, "pickup_conflict", goals)
                    collected.update(removed)
                    frames.append((trial.pose, list(trial.collected), 0., "pickup_stop"))
                    pickups.append({"goal_index": index, "ids": sorted(removed), "pose": trial.pose,
                                    "trace_index": start_index + len(frames)})
        except ValueError as error:
            return self._reject(stage, str(error), goals)
        if stage == "pickup" and collected != allowed_ids:
            return self._reject(stage, "incomplete_batch", goals)
        trial.motion_circle = original_circle
        self._scene = trial
        for pose, ids, elapsed, phase in frames:
            self.time = round(self.time + elapsed, 10)
            self.trace.append({"time_s": self.time, "pose": pose, "stage": phase,
                               "auto_fence": circle, "collected_ids": ids})
        result = {"stage": stage, "status": "success", "reason": None, "start_pose": start_pose,
                  "end_pose": self.pose, "trace_start": start_index, "trace_end": len(self.trace) - 1,
                  "goals": goals, "pickups": pickups}
        self.actions.append(result)
        return result

    def final_state(self):
        return self._scene.snapshot()
