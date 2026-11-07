"""Point-target rules independent of IR-SIM and the graphical interface.

Actions are straight translations or rotations in place. Rectangle boundary
crossing times are solved analytically, including tangencies, so checking only
the displayed animation frames cannot hide a collision or collection.
"""

from __future__ import annotations

from dataclasses import dataclass
from copy import copy
from importlib.resources import files
import math
import random
from pathlib import Path

from .config import load_config

EPS = 1e-9
DEFAULT_CONFIG = files(__package__).joinpath("world.yaml")


def finite(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Values must be finite numbers")
    return value


def normalize(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi


def trig_times(a: float, b: float, value: float, delta: float) -> list[float]:
    """Solve a*cos(delta*t) + b*sin(delta*t) = value, 0 <= t <= 1."""
    radius = math.hypot(a, b)
    if radius < EPS or abs(delta) < EPS or abs(value) > radius + EPS:
        return []
    phase = math.atan2(b, a)
    alpha = math.acos(max(-1.0, min(1.0, value / radius)))
    low, high = sorted((0.0, delta))
    result = []
    for root in (phase - alpha, phase + alpha):
        for k in range(math.ceil((low - root - EPS) / math.tau),
                       math.floor((high - root + EPS) / math.tau) + 1):
            t = (root + k * math.tau) / delta
            if -EPS <= t <= 1 + EPS:
                result.append(max(0.0, min(1.0, t)))
    return result


@dataclass(frozen=True)
class Rect:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def contains(self, x: float, y: float) -> bool:
        return (self.xmin - EPS <= x <= self.xmax + EPS
                and self.ymin - EPS <= y <= self.ymax + EPS)

    @property
    def corners(self) -> list[tuple[float, float]]:
        return [(self.xmin, self.ymin), (self.xmax, self.ymin),
                (self.xmax, self.ymax), (self.xmin, self.ymax)]

    def first_contact(self, px: float, py: float, distance: float,
                      rotation: float) -> float:
        candidates = [0.0, 1.0]
        if distance:
            candidates += [(px - edge) / distance for edge in (self.xmin, self.xmax)]
        elif rotation:
            for edge in (self.xmin, self.xmax):
                candidates += trig_times(px, py, edge, rotation)
            for edge in (self.ymin, self.ymax):
                candidates += trig_times(py, -px, edge, rotation)
        for t in sorted(candidates):
            if not -EPS <= t <= 1 + EPS:
                continue
            t = max(0.0, min(1.0, t))
            c, s = math.cos(rotation * t), math.sin(rotation * t)
            if self.contains(c * px + s * py - distance * t, -s * px + c * py):
                return t
        return math.inf


class Scene:
    """Episode API. Targets are mathematical points; rectangle edges count.

    `turn_to(degrees)` and `forward(metres)` execute a complete action. The pose
    controller uses `advance` for animation. The expanded body, including its
    boundary, is checked continuously; on collision the scene freezes.
    """

    def __init__(self, config: str | Path | None = None, scenario: str = "front",
                 seed: int | None = None, body_margin: float | None = None,
                 robot_config: str | Path | None = None):
        self.config_path = (Path(config) if config else Path(str(DEFAULT_CONFIG))).resolve()
        self.config, self.robot_config_path = load_config(self.config_path, robot_config)
        world, robot, custom = (self.config[k] for k in ("world", "robot", "custom"))
        self.width, self.height = finite(world["width"]), finite(world["height"])
        if finite(world["step_time"]) <= 0:
            raise ValueError("step_time must be positive")
        length, width = finite(robot["shape"]["length"]), finite(robot["shape"]["width"])
        depth, cw = finite(custom["collector"]["depth"]), finite(custom["collector"]["width"])
        if min(self.width, self.height, length, width, depth, cw) <= 0:
            raise ValueError("Map, body and collector dimensions must be positive")
        if robot["shape"]["name"] != "rectangle":
            raise ValueError("This scene requires a rectangular robot")
        if world.get("offset", [0, 0]) != [0, 0]:
            raise ValueError("This scene requires world.offset = [0, 0]")
        self.body = Rect(-length / 2, -width / 2, length / 2, width / 2)
        self.collector_gap = finite(custom["collector"]["gap"])
        self.collector_offset = finite(custom["collector"]["lateral_offset"])
        front = length / 2 + self.collector_gap
        self.collector = Rect(front, self.collector_offset - cw / 2,
                              front + depth, self.collector_offset + cw / 2)
        self.body_margin = finite(custom.get("body_margin", 0) if body_margin is None else body_margin)
        if self.body_margin < 0:
            raise ValueError("Body margin must be nonnegative")
        custom["body_margin"] = self.body_margin
        self.initial_pose = tuple(finite(v) for v in robot["state"])
        if len(self.initial_pose) != 3:
            raise ValueError("Robot state must contain x, y, theta")
        self.seed = int(custom["seed"] if seed is None else seed)
        for key in ("forward_speed", "angular_speed"):
            if finite(custom[key]) <= 0:
                raise ValueError(f"{key} must be positive")
        self.reset(scenario)

    def local_point(self, x: float, y: float) -> tuple[float, float]:
        px, py, theta = self.pose
        c, s = math.cos(theta), math.sin(theta)
        dx, dy = x - px, y - py
        return c * dx + s * dy, -s * dx + c * dy

    def vertices(self, rect: Rect) -> list[tuple[float, float]]:
        px, py, theta = self.pose
        c, s = math.cos(theta), math.sin(theta)
        return [(px + c * x - s * y, py + s * x + c * y) for x, y in rect.corners]

    @property
    def safety_body(self) -> Rect:
        margin = self.body_margin
        return Rect(self.body.xmin - margin, self.body.ymin - margin,
                    self.body.xmax + margin, self.body.ymax + margin)

    @property
    def dimensions(self) -> dict[str, float]:
        return {"body_length": self.body.xmax - self.body.xmin,
                "body_width": self.body.ymax - self.body.ymin,
                "collector_width": self.collector.ymax - self.collector.ymin,
                "collector_depth": self.collector.xmax - self.collector.xmin,
                "body_margin": self.body_margin}

    def set_dimensions(self, *, body_length: float, body_width: float,
                       collector_width: float, collector_depth: float,
                       body_margin: float | None = None) -> None:
        """Resize in place, committing only if the new footprint is valid.

        Retain pose, path and targets. Newly covered collector targets are
        collected immediately; a safety-body overlap is rejected instead of deleting
        the target. Both current and reset poses must remain inside the map.
        """
        if self.failure:
            raise ValueError("Reset the scene after a collision")
        length, width, cw, depth = map(finite, (body_length, body_width,
                                               collector_width, collector_depth))
        if min(length, width, cw, depth) <= 0:
            raise ValueError("Dimensions must be positive")
        margin = self.body_margin if body_margin is None else finite(body_margin)
        if margin < 0:
            raise ValueError("Body margin must be nonnegative")
        body = Rect(-length / 2, -width / 2, length / 2, width / 2)
        front = length / 2 + self.collector_gap
        collector = Rect(front, self.collector_offset - cw / 2,
                          front + depth, self.collector_offset + cw / 2)
        candidate = copy(self)
        candidate.body, candidate.collector = body, collector
        candidate.body_margin = margin
        for pose in (self.pose, self.initial_pose):
            candidate.pose = pose
            if candidate._wall_contact(0, 0) < math.inf:
                raise ValueError("Resized footprint must fit inside the map")
        if any(candidate.safety_body.contains(*self.local_point(x, y)) for x, y in self.targets.values()):
            raise ValueError("Resized body overlaps an uncollected target")
        self.body, self.collector = body, collector
        self.body_margin = margin
        self.config["robot"]["shape"].update(length=length, width=width)
        self.config["custom"]["collector"].update(width=cw, depth=depth)
        self.config["custom"]["body_margin"] = margin
        for key, point in list(self.targets.items()):
            if collector.contains(*self.local_point(*point)):
                self.collected[key] = self.targets.pop(key)

    def reset(self, scenario: str | None = None) -> None:
        # A different footprint can make a preset invalid. Keep the previous
        # episode intact if generating the requested scene fails.
        candidate = copy(self)
        candidate._reset(scenario)
        self.__dict__.update(candidate.__dict__)

    def _reset(self, scenario: str | None = None) -> None:
        self.scenario = scenario or getattr(self, "scenario", "front")
        self.pose = self.initial_pose
        self.targets: dict[int, tuple[float, float]] = {}
        self.collected: dict[int, tuple[float, float]] = {}
        self.failure: str | None = None
        self.collision_point: tuple[float, float] | None = None
        self.path = [self.pose[:2]]
        self._next_id = 1
        if self._wall_contact(0, 0) < math.inf:
            raise ValueError("Initial robot and collector must fit strictly inside the map")
        x, y, theta = self.initial_pose
        c, s = math.cos(theta), math.sin(theta)
        if self.scenario == "front":
            positions = [(1.8, -.22), (1.9, .16), (2.1, 0),
                         (3.1, 1.0), (3.3, 1.15), (3.45, .9),
                         (4.2, -1.2), (4.4, -1.05), (5.0, .2), (5.2, .4)]
        elif self.scenario == "surround":
            positions = [(r * math.cos(a), r * math.sin(a))
                         for r in (1.3, 2.1)
                         for a in (i * math.tau / 16 for i in range(16))]
        elif self.scenario == "random":
            positions = None
        else:
            raise ValueError(f"Unknown scenario: {self.scenario}")
        if positions is not None:
            for dx, dy in positions:
                self.add_target(x + c * dx - s * dy, y + s * dx + c * dy)
        else:
            rng = random.Random(self.seed)
            count = int(self.config["custom"]["random_count"])
            if count < 0:
                raise ValueError("random_count must be nonnegative")
            attempts = 0
            while len(self.targets) < count and attempts < max(1000, count * 100):
                attempts += 1
                tx, ty = rng.uniform(.1, self.width - .1), rng.uniform(.1, self.height - .1)
                lx, ly = self.local_point(tx, ty)
                if not self.safety_body.contains(lx, ly) and not self.collector.contains(lx, ly):
                    self.add_target(tx, ty)
            if len(self.targets) != count:
                raise ValueError("Not enough space to place random targets")

    def add_target(self, x: float, y: float) -> None:
        if self.failure:
            raise ValueError("Reset the scene after a collision")
        x, y = finite(x), finite(y)
        if not (0 < x < self.width and 0 < y < self.height):
            raise ValueError("Target must be inside the map")
        local = self.local_point(x, y)
        if self.safety_body.contains(*local):
            raise ValueError("Cannot place a target inside the robot body")
        if any(math.dist((x, y), p) < .025 for p in self.targets.values()):
            return
        collection = self.collected if self.collector.contains(*local) else self.targets
        collection[self._next_id] = (x, y)
        self._next_id += 1

    def remove_nearest(self, x: float, y: float, radius: float = .25) -> None:
        if self.targets:
            key = min(self.targets, key=lambda k: math.dist((x, y), self.targets[k]))
            if math.dist((x, y), self.targets[key]) <= radius:
                del self.targets[key]

    def _wall_contact(self, distance: float, rotation: float) -> float:
        x, y, theta = self.pose
        first = math.inf
        for rect in (self.safety_body, self.collector):
            for wx, wy in self.vertices(rect):
                if wx <= EPS or wx >= self.width - EPS or wy <= EPS or wy >= self.height - EPS:
                    return 0.0
                dx, dy = wx - x, wy - y
                if distance:
                    vx, vy = distance * math.cos(theta), distance * math.sin(theta)
                    times = [(edge - wx) / vx for edge in (0, self.width)] if abs(vx) > EPS else []
                    times += [(edge - wy) / vy for edge in (0, self.height)] if abs(vy) > EPS else []
                else:
                    times = []
                    for edge in (0, self.width):
                        times += trig_times(dx, -dy, edge - x, rotation)
                    for edge in (0, self.height):
                        times += trig_times(dy, dx, edge - y, rotation)
                first = min([first] + [max(0.0, t) for t in times if -EPS <= t <= 1 + EPS])
        return first

    def advance(self, *, distance: float = 0, rotation: float = 0) -> None:
        distance, rotation = finite(distance), finite(rotation)
        if distance < 0:
            raise ValueError("Reverse motion is disabled")
        if distance and rotation:
            raise ValueError("Turn in place or move forward; execute them separately")
        if self.failure:
            return
        stop = self._wall_contact(distance, rotation)
        failure = "Map boundary" if math.isfinite(stop) else None
        collision_point = None
        collections = []
        for key, (x, y) in self.targets.items():
            local = self.local_point(x, y)
            body_time = self.safety_body.first_contact(*local, distance, rotation)
            collect_time = self.collector.first_contact(*local, distance, rotation)
            if collect_time + EPS < body_time:
                collections.append((collect_time, key))
            elif body_time < stop:
                stop, failure, collision_point = body_time, f"Body hit target #{key}", (x, y)
        for time, key in sorted(collections):
            if math.isfinite(time) and time <= stop + EPS:
                self.collected[key] = self.targets.pop(key)
        t = min(1.0, stop)
        x, y, theta = self.pose
        self.pose = (x + distance * t * math.cos(theta),
                     y + distance * t * math.sin(theta), normalize(theta + rotation * t))
        if self.pose[:2] != self.path[-1]:
            self.path.append(self.pose[:2])
        self.failure, self.collision_point = failure, collision_point

    def turn_to(self, degrees: float) -> None:
        self.advance(rotation=normalize(math.radians(finite(degrees)) - self.pose[2]))

    def forward(self, metres: float) -> None:
        self.advance(distance=metres)

    def snapshot(self) -> dict:
        return {"pose": list(self.pose), "remaining": dict(self.targets),
                "collected": dict(self.collected), "failure": self.failure,
                "scenario": self.scenario, "seed": self.seed,
                "dimensions": self.dimensions}
