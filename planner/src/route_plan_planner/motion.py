"""All steering and replay use the simulator's public pose controller."""

from dataclasses import dataclass
import math
from typing import Protocol

from route_plan_sim import PoseController, Scene
from route_plan_sim.scene import EPS, normalize

Pose = tuple[float, float, float]


@dataclass
class Transition:
    scene: Scene
    goals: list[Pose]
    source: str = "direct"


class MotionBackend(Protocol):
    """A connector may collect targets; its returned state must reflect that."""

    def connect(self, scene: Scene, goal: Pose, deadline: float) -> Transition | None: ...


def execute(scene: Scene, goals: list[Pose], dt: float | None = None,
            pickup_stops: list[dict] | None = None) -> Scene:
    """Mutate only the supplied episode. dt=None executes entire pose commands."""
    controller = PoseController(scene)
    stops = {stop["goal_index"]: stop for stop in pickup_stops or []}
    if len(stops) != len(pickup_stops or []) or any(type(i) is not int or not 0 <= i < len(goals) for i in stops):
        raise ValueError("Invalid pickup stop goal indices")
    for index, goal in enumerate(goals):
        if scene.failure:
            break
        controller.set_goal(*goal)
        if dt is None:
            custom = scene.config["custom"]
            # Upper bound includes approach and final rotations at configured speeds.
            duration = (math.hypot(scene.width, scene.height) / custom["forward_speed"]
                        + 2 * math.tau / custom["angular_speed"] + 1)
            controller.step(duration)
        else:
            if not math.isfinite(dt) or dt <= 0:
                raise ValueError("Replay dt must be positive and finite")
            while controller.command is not None and not scene.failure:
                controller.step(dt)
        if not scene.failure and index in stops:
            removed = scene.collect_at_stop()
            if set(removed) != set(stops[index]["target_ids"]):
                raise ValueError("Pickup stop coverage differs from the plan")
    return scene


def replay(scene: Scene, goals: list[Pose], dt: float = .04,
           pickup_stops: list[dict] | None = None) -> Scene:
    """Independent, frame-by-frame execution; the caller's scene is untouched."""
    return execute(scene.clone(), goals, dt, pickup_stops)


def _turn(scene: Scene, yaw: float, long_way: bool = False) -> Transition | None:
    delta = normalize(yaw - scene.pose[2])
    if long_way and abs(delta) < EPS:
        return None
    if long_way:
        delta -= math.copysign(math.tau, delta)
    trial = scene.clone()
    goals = []
    pieces = max(1, math.ceil(abs(delta) / (math.pi / 2)))
    initial_yaw = scene.pose[2]
    for index in range(1, pieces + 1):
        goal = (*trial.pose[:2], normalize(initial_yaw + delta * index / pieces))
        execute(trial, [goal])
        goals.append(goal)
        if trial.failure:
            return None
    return Transition(trial, goals)


def steer(scene: Scene, goal: Pose) -> Transition | None:
    """Turn, move forward, turn. Try the other turn direction if needed.

    A rear location is reached only after legal same-position yaw commands.
    Long rotations are split into <=90-degree pose commands, never reverse.
    """
    gx, gy, yaw = goal
    if not (0 < gx < scene.width and 0 < gy < scene.height):
        return None
    dx, dy = gx - scene.pose[0], gy - scene.pose[1]
    bearing = math.atan2(dy, dx) if math.hypot(dx, dy) > EPS else scene.pose[2]
    for long_first in (False, True):
        first = _turn(scene, bearing, long_first)
        if first is None:
            continue
        drive_goal = (gx, gy, bearing)
        execute(first.scene, [drive_goal])
        if first.scene.failure:
            continue
        for long_last in (False, True):
            last = _turn(first.scene, yaw, long_last)
            if last is not None:
                return Transition(last.scene, first.goals + [drive_goal] + last.goals)
    return None


class DirectMotion:
    def connect(self, scene: Scene, goal: Pose, deadline: float) -> Transition | None:
        direct = steer(scene, goal)
        if direct is not None:
            return direct
        # Near a wall, driving to the collection pose and only then turning can
        # fail. Align at a staging pose first, then approach along the tool axis.
        x, y, yaw = goal
        distance = max(scene.dimensions["body_length"], scene.dimensions["collector_depth"])
        staging = (x - distance * math.cos(yaw), y - distance * math.sin(yaw), yaw)
        first = steer(scene, staging)
        if first is None:
            return None
        last = steer(first.scene, goal)
        if last is None:
            return None
        return Transition(last.scene, first.goals + last.goals, "staged")


def path_length(start: Pose, goals: list[Pose]) -> float:
    total = 0.
    for goal in goals:
        total += math.dist(start[:2], goal[:2])
        start = goal
    return total


def compact_goals(start: Pose, goals: list[Pose]) -> list[Pose]:
    result = []
    for goal in goals:
        if math.dist(start[:2], goal[:2]) > EPS or abs(normalize(goal[2] - start[2])) > EPS:
            result.append(goal)
            start = goal
    return result
