"""Circle-target rules independent of IR-SIM and the graphical interface.

Actions are straight translations or rotations in place. Circle/rectangle
contact times are solved analytically, including corner tangencies, so checking
only displayed animation frames cannot hide a collision or collection.
"""

from __future__ import annotations

from dataclasses import dataclass
from copy import copy, deepcopy
from importlib.resources import files
import math
import random
from pathlib import Path

from .boundary import EPS, Fence, trig_times
from .config import load_config

DEFAULT_CONFIG = files(__package__).joinpath("world.yaml")


def finite(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Values must be finite numbers")
    return value


def normalize(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi


@dataclass(frozen=True)
class Rect:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def contains(self, x: float, y: float) -> bool:
        return (self.xmin - EPS <= x <= self.xmax + EPS
                and self.ymin - EPS <= y <= self.ymax + EPS)

    def intersects_circle(self, x: float, y: float, radius: float) -> bool:
        """Closed rectangle vs disk, including rounded corner offsets."""
        dx = max(self.xmin - x, 0., x - self.xmax)
        dy = max(self.ymin - y, 0., y - self.ymax)
        return math.hypot(dx, dy) <= radius + EPS

    @property
    def corners(self) -> list[tuple[float, float]]:
        return [(self.xmin, self.ymin), (self.xmax, self.ymin),
                (self.xmax, self.ymax), (self.xmin, self.ymax)]

    def first_contact(self, px: float, py: float, distance: float,
                      rotation: float, radius: float = 0.) -> float:
        """Solve contacts with the offset rectangle's sides and corner arcs.

        Radius zero retains the historical point model for archived records.
        """
        candidates = [0.0, 1.0]
        if distance:
            candidates += [(px - edge) / distance for edge in (self.xmin - radius, self.xmax + radius)]
            if radius:
                for cx, cy in self.corners:
                    dy = py - cy
                    if abs(dy) <= radius + EPS:
                        root = math.sqrt(max(0., radius**2 - dy**2))
                        candidates += [(px - cx - root) / distance, (px - cx + root) / distance]
        elif rotation:
            for edge in (self.xmin - radius, self.xmax + radius):
                candidates += trig_times(px, py, edge, rotation)
            for edge in (self.ymin - radius, self.ymax + radius):
                candidates += trig_times(py, -px, edge, rotation)
            if radius:
                for cx, cy in self.corners:
                    # |R(-rotation*t)*p - corner|² = radius².
                    value = (px**2 + py**2 + cx**2 + cy**2 - radius**2) / 2
                    candidates += trig_times(px * cx + py * cy, py * cx - px * cy, value, rotation)
        for t in sorted(candidates):
            if not -EPS <= t <= 1 + EPS:
                continue
            t = max(0.0, min(1.0, t))
            c, s = math.cos(rotation * t), math.sin(rotation * t)
            if self.intersects_circle(c * px + s * py - distance * t, -s * px + c * py, radius):
                return t
        return math.inf

    def turn_preserves_clearance(self, px, py, angle, contact_distance=None):
        """Check clearance at face changes and distance extrema during a turn."""
        times = {0., 1.}
        for edge in (self.xmin, self.xmax, 0.):
            times.update(trig_times(px, py, edge, angle))
        for edge in (self.ymin, self.ymax, 0.):
            times.update(trig_times(py, -px, edge, angle))
        for cx, cy in self.corners:
            times.update(trig_times(cx * py - cy * px, -cx * px - cy * py, 0., angle))
        largest = 0.
        for t in sorted(times):
            c, s = math.cos(angle * t), math.sin(angle * t)
            x, y = c * px + s * py, -s * px + c * py
            distance = math.hypot(max(self.xmin - x, 0., x - self.xmax),
                                  max(self.ymin - y, 0., y - self.ymax))
            if distance + EPS < largest:
                return False
            if contact_distance is not None and distance <= contact_distance < largest:
                return False
            largest = max(distance, largest)
        return True


class Scene:
    """Episode API. Targets are disks; touching rectangle edges counts.

    `turn_to(degrees)` and `forward(metres)` execute a complete action. The pose
    controller uses `advance` for animation. The expanded body, including its
    boundary, is checked continuously; on collision the scene freezes.
    """

    def __init__(self, config: str | Path | None = None, scenario: str = "front",
                 seed: int | None = None, body_margin: float | None = None,
                 robot_config: str | Path | None = None, target_diameter: float | None = None,
                 *, obstacles=None):
        self.config_path = (Path(config) if config else Path(str(DEFAULT_CONFIG))).resolve()
        self.config, self.robot_config_path = load_config(self.config_path, robot_config)
        world, robot, custom = (self.config[k] for k in ("world", "robot", "custom"))
        # Keep the mounting rules of archived plans.
        self.rule_version = custom.get("rule_version", 3)
        if type(self.rule_version) is not int or self.rule_version not in (1, 2, 3):
            raise ValueError("Unsupported rule_version")
        custom["rule_version"] = self.rule_version
        self.collection_mode = custom.get("collection_mode", "contact")
        if self.collection_mode not in ("contact", "at_stop"):
            raise ValueError("Unsupported collection_mode")
        custom["collection_mode"] = self.collection_mode
        self.pickup_rule = custom.get("pickup_rule", "center" if self.collection_mode == "at_stop" else "contact")
        if self.pickup_rule not in ("contact", "center"):
            raise ValueError("Unsupported pickup_rule")
        custom["pickup_rule"] = self.pickup_rule
        self.width, self.height = finite(world["width"]), finite(world["height"])
        self.fence_enabled = custom.get("fence_enabled", True)
        if not isinstance(self.fence_enabled, bool):
            raise ValueError("fence_enabled must be boolean")
        points = custom.get("fence")
        self.fence = Fence.from_points(points) if self.fence_enabled and points is not None else None
        motion_circle = custom.get("motion_circle")
        self.motion_circle = None
        if motion_circle is not None:
            cx, cy, radius = map(finite, motion_circle)
            if radius <= 0.:
                raise ValueError("Motion circle radius must be positive")
            self.motion_circle = (cx, cy, radius)
        if self.fence is not None:
            xmin, ymin, xmax, ymax = self.fence.bounds
            self.width, self.height = xmax - xmin, ymax - ymin
            self.map_bounds = self.fence.bounds
        else:
            xmin, ymin = map(finite, world.get("offset", [0, 0]))
            self.map_bounds = (xmin, ymin, xmin + self.width, ymin + self.height)
        if finite(world["step_time"]) <= 0:
            raise ValueError("step_time must be positive")
        self.target_diameter = finite(custom.get("target_diameter", .1)
                                      if target_diameter is None else target_diameter)
        if self.target_diameter < 0 or self.target_diameter >= min(self.width, self.height):
            raise ValueError("Target diameter must be nonnegative and smaller than the map")
        custom["target_diameter"] = self.target_diameter
        length, width = finite(robot["shape"]["length"]), finite(robot["shape"]["width"])
        depth, cw = finite(custom["collector"]["depth"]), finite(custom["collector"]["width"])
        if min(self.width, self.height, length, width, depth, cw) <= 0:
            raise ValueError("Map, body and collector dimensions must be positive")
        obstacle_values = custom.get("obstacles", ()) if obstacles is None else obstacles
        self.obstacles = tuple(tuple(map(finite, disk)) for disk in obstacle_values)
        xmin, ymin, xmax, ymax = self.map_bounds
        for x, y, radius in self.obstacles:
            inside = (self.fence.contains_disk(x, y, radius) if self.fence is not None
                      else radius < min(x - xmin, y - ymin, xmax - x, ymax - y))
            if radius <= 0 or not inside:
                raise ValueError("Obstacle disks must have positive radius and fit inside the map")
        if self.obstacles or "obstacles" in custom:
            custom["obstacles"] = [list(disk) for disk in self.obstacles]
        if robot["shape"]["name"] != "rectangle":
            raise ValueError("This scene requires a rectangular robot")
        self.body = Rect(-length / 2, -width / 2, length / 2, width / 2)
        self.body_margin = finite(custom.get("body_margin", 0) if body_margin is None else body_margin)
        if self.body_margin < 0:
            raise ValueError("Body margin must be nonnegative")
        custom["body_margin"] = self.body_margin
        self._gap_follows_margin = self.rule_version == 2 or (
            self.rule_version == 3 and "gap" not in custom["collector"]
        )
        self.collector_gap = (self.body_margin if self._gap_follows_margin
                              else finite(custom["collector"].get("gap", 0)))
        custom["collector"]["gap"] = self.collector_gap
        self.collector_offset = finite(custom["collector"]["lateral_offset"])
        front = length / 2 + self.collector_gap
        self.collector = Rect(front, self.collector_offset - cw / 2,
                              front + depth, self.collector_offset + cw / 2)
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

    def contains_disk(self, x: float, y: float, radius: float = 0.) -> bool:
        if self.fence is not None:
            return self.fence.contains_disk(x, y, radius)
        xmin, ymin, xmax, ymax = self.map_bounds
        inset = radius + EPS
        return xmin + inset < x < xmax - inset and ymin + inset < y < ymax - inset

    def contains_motion_point(self, x: float, y: float) -> bool:
        """Check the optional motion boundary for the robot reference point."""
        if self.motion_circle is None:
            return True
        cx, cy, radius = self.motion_circle
        return math.hypot(x - cx, y - cy) <= radius + EPS

    def clone(self) -> Scene:
        """Copy an episode for speculative planning, without reloading files.

        Mutable configuration, targets, collected targets and trajectory are
        independent. Immutable rectangles and tuple poses can safely be shared.
        This does not construct a viewer or change the original episode.
        """
        result = copy(self)
        result.config = deepcopy(self.config)
        result.targets = self.targets.copy()
        result.collected = self.collected.copy()
        result.path = self.path.copy()
        result.collection_events = self.collection_events.copy()
        return result

    def vertices(self, rect: Rect) -> list[tuple[float, float]]:
        px, py, theta = self.pose
        c, s = math.cos(theta), math.sin(theta)
        return [(px + c * x - s * y, py + s * x + c * y) for x, y in rect.corners]

    @property
    def target_radius(self) -> float:
        return self.target_diameter / 2

    @property
    def pickup_radius(self) -> float:
        # Pickup acceptance uses the centre in the new stop model. The real
        # disk radius is still used, independently, for every body collision.
        return 0. if self.pickup_rule == "center" else self.target_radius

    @property
    def target_groups(self) -> tuple[tuple[int, ...], ...]:
        """Connected overlapping disks form one atomic collection task.

        Keep every disk for collision checks; never shrink a group to its
        centroid. The immutable cache can safely be shared by scene clones.
        Tangent disks remain separate; coincident centres share a group.
        """
        signature = (self.target_diameter, tuple(self.targets.items()))
        cached = getattr(self, "_target_groups_cache", None)
        if cached is not None and cached[0] == signature:
            return cached[1]
        parents = {key: key for key in self.targets}

        def root(key):
            while parents[key] != key:
                parents[key] = parents[parents[key]]
                key = parents[key]
            return key

        items = list(self.targets.items())
        for index, (key, point) in enumerate(items):
            for other, position in items[:index]:
                if point == position or math.dist(point, position) < self.target_diameter - EPS:
                    parents[root(key)] = root(other)
        groups = {}
        for key in self.targets:
            groups.setdefault(root(key), []).append(key)
        result = tuple(tuple(group) for group in groups.values())
        self._target_groups_cache = signature, result
        return result

    def _collect_at_pose(self) -> None:
        if self.pickup_rule == "center":
            for key, point in list(self.targets.items()):
                if self.collector.contains(*self.local_point(*point)):
                    self.collected[key] = self.targets.pop(key)
            return
        for group in self.target_groups:
            if any(self.collector.intersects_circle(*self.local_point(*self.targets[key]), self.target_radius)
                   for key in group):
                for key in group:
                    self.collected[key] = self.targets.pop(key)

    def collect_at_stop(self) -> tuple[int, ...]:
        """Collect only disks covered at this single pose, after collision checks."""
        if self.failure:
            raise ValueError("Cannot collect after a collision")
        if self.recovering:
            raise ValueError("Cannot collect before clearing the initial safety intrusions")
        before = set(self.collected)
        self.advance()
        if self.failure:
            raise ValueError("Cannot collect at an unsafe stop")
        self._collect_at_pose()
        return tuple(key for key in self.collected if key not in before)

    @property
    def safety_body(self) -> Rect:
        margin = self.body_margin
        return Rect(self.body.xmin - margin, self.body.ymin - margin,
                    self.body.xmax + margin, self.body.ymax + margin)

    @property
    def recovering(self) -> bool:
        return bool(self._initial_intrusions & self.targets.keys())

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
        gap = margin if self._gap_follows_margin else self.collector_gap
        front = length / 2 + gap
        collector = Rect(front, self.collector_offset - cw / 2,
                          front + depth, self.collector_offset + cw / 2)
        candidate = copy(self)
        candidate.body, candidate.collector = body, collector
        candidate.body_margin = margin
        for pose in (self.pose, self.initial_pose):
            candidate.pose = pose
            if candidate._wall_contact(0, 0) < math.inf:
                raise ValueError("Resized footprint must fit inside the map")
            if math.isfinite(candidate._obstacle_contact(0, 0)[0]):
                raise ValueError("Resized footprint touches an obstacle")
        if any(candidate.safety_body.intersects_circle(*self.local_point(x, y), self.target_radius)
               for x, y in self.targets.values()):
            raise ValueError("Resized body overlaps an uncollected target")
        self.body, self.collector = body, collector
        self.body_margin = margin
        self.collector_gap = gap
        self.config["robot"]["shape"].update(length=length, width=width)
        self.config["custom"]["collector"].update(width=cw, depth=depth, gap=gap)
        self.config["custom"]["body_margin"] = margin
        if self.collection_mode == "contact":
            self._collect_at_pose()

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
        self._initial_intrusions: frozenset[int] = frozenset()
        self.collected: dict[int, tuple[float, float]] = {}
        self.failure: str | None = None
        self.collision_point: tuple[float, float] | None = None
        self.path = [self.pose[:2]]
        self._next_id = 1
        self.collection_events = []
        if not self.contains_motion_point(*self.pose[:2]):
            raise ValueError("Initial robot position must be inside the motion circle")
        if self._wall_contact(0, 0) < math.inf:
            raise ValueError("Initial robot and collector must fit strictly inside the map")
        if math.isfinite(self._obstacle_contact(0, 0)[0]):
            raise ValueError("Initial footprint touches an obstacle")
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
        elif self.scenario == "empty":
            positions = ()
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
                inset = max(.1, self.target_radius + 2 * EPS)
                xmin, ymin, xmax, ymax = self.map_bounds
                tx, ty = rng.uniform(xmin + inset, xmax - inset), rng.uniform(ymin + inset, ymax - inset)
                lx, ly = self.local_point(tx, ty)
                if (not self.safety_body.intersects_circle(lx, ly, self.target_radius)
                        and not self.collector.intersects_circle(lx, ly, self.target_radius)
                        and not self._overlaps_target(tx, ty)
                        and self.contains_disk(tx, ty, self.target_radius)
                        and not self._overlaps_obstacle(tx, ty)):
                    self.add_target(tx, ty)
            if len(self.targets) != count:
                raise ValueError("Not enough space to place random targets")

    def add_target(self, x: float, y: float) -> None:
        self.add_targets([(x, y)])

    def add_targets(self, points, *, allow_safety_intrusions: bool = False) -> tuple[int, ...]:
        """Atomically add an observed batch, then collect touching groups.

        Validate every member before initial collection, independently of input
        order. Existing collected disks are absent and do not join new groups.
        """
        if self.failure:
            raise ValueError("Reset the scene after a collision")
        if allow_safety_intrusions and (self.collection_mode != "at_stop" or self.pickup_rule != "center"
                                        or self.targets or self.pose != self.initial_pose):
            raise ValueError("Recovery targets require a fresh centre-rule pickup scene")
        points = tuple((finite(x), finite(y)) for x, y in points)
        radius = self.target_radius
        intrusions = set()
        for index, (x, y) in enumerate(points):
            if not self.contains_disk(x, y, radius):
                raise ValueError("Target must be inside the map")
            if self._overlaps_obstacle(x, y):
                raise ValueError("Target touches an obstacle")
            local = self.local_point(x, y)
            if self.safety_body.intersects_circle(*local, radius):
                if not allow_safety_intrusions or self.body.intersects_circle(*local, radius):
                    raise ValueError("Cannot place a target inside the robot body")
                intrusions.add(self._next_id + index)
        keys = tuple(range(self._next_id, self._next_id + len(points)))
        self.targets.update(zip(keys, points))
        self._initial_intrusions |= intrusions
        self._next_id += len(points)
        if self.collection_mode == "contact":
            self._collect_at_pose()
        return keys

    def _overlaps_target(self, x: float, y: float) -> bool:
        return any(math.dist((x, y), p) < self.target_diameter - EPS for p in self.targets.values())

    def _overlaps_obstacle(self, x: float, y: float) -> bool:
        return any(math.hypot(x - ox, y - oy) <= radius + self.target_radius + EPS
                   for ox, oy, radius in self.obstacles)

    def remove_nearest(self, x: float, y: float, radius: float = .25) -> None:
        if self.targets:
            key = min(self.targets, key=lambda k: math.dist((x, y), self.targets[k]))
            if math.dist((x, y), self.targets[key]) <= radius:
                del self.targets[key]

    def _wall_contact(self, distance: float, rotation: float) -> float:
        if self.fence is not None:
            return min(self.fence.first_contact(rect, self.pose, distance, rotation)
                       for rect in (self.safety_body, self.collector))
        x, y, theta = self.pose
        xmin, ymin, xmax, ymax = self.map_bounds
        x_edges, y_edges = (xmin, xmax), (ymin, ymax)
        first = math.inf
        for rect in (self.safety_body, self.collector):
            for wx, wy in self.vertices(rect):
                if not (xmin + EPS < wx < xmax - EPS and ymin + EPS < wy < ymax - EPS):
                    return 0.0
                dx, dy = wx - x, wy - y
                if distance:
                    vx, vy = distance * math.cos(theta), distance * math.sin(theta)
                    times = [(edge - wx) / vx for edge in x_edges] if abs(vx) > EPS else []
                    times += [(edge - wy) / vy for edge in y_edges] if abs(vy) > EPS else []
                else:
                    times = []
                    for edge in x_edges:
                        times += trig_times(dx, -dy, edge - x, rotation)
                    for edge in y_edges:
                        times += trig_times(dy, dx, edge - y, rotation)
                first = min([first] + [max(0.0, t) for t in times if -EPS <= t <= 1 + EPS])
        return first

    def _motion_circle_exit(self, distance: float) -> float:
        if self.motion_circle is None:
            return math.inf
        x, y, yaw = self.pose
        if not self.contains_motion_point(x, y):
            return 0.
        c, s = math.cos(yaw), math.sin(yaw)
        if not distance or self.contains_motion_point(x + distance * c, y + distance * s):
            # A circle is convex and a turn keeps the reference point fixed.
            return math.inf
        cx, cy, radius = self.motion_circle
        dx, dy = x - cx, y - cy
        along, across = c * dx + s * dy, c * dy - s * dx
        radial_fraction = across / radius
        exit_distance = radius * math.sqrt(max(0., 1. - radial_fraction * radial_fraction)) - along
        return max(0., min(1., exit_distance / distance))

    def _initial_intrusion_contact(self, x, y, distance, rotation):
        """Allow an initially overlapping disk only while moving away from it."""
        if distance and x > self.body.xmax:
            return 0.
        if rotation and (not self.body.turn_preserves_clearance(x, y, rotation)
                         or not self.safety_body.turn_preserves_clearance(x, y, rotation, self.target_radius + EPS)):
            return 0.
        return self.body.first_contact(x, y, distance, rotation, self.target_radius)

    def _obstacle_contact(self, distance, rotation):
        first = (math.inf, 0)
        footprints = (self.safety_body, self.collector)
        for index, (x, y, radius) in enumerate(self.obstacles, 1):
            local = self.local_point(x, y)
            for rect in footprints:
                first = min(first, (rect.first_contact(*local, distance, rotation, radius), index))
        return first

    def advance(self, *, distance: float = 0, rotation: float = 0) -> None:
        distance, rotation = finite(distance), finite(rotation)
        if distance < 0:
            raise ValueError("Reverse motion is disabled")
        if distance and rotation:
            raise ValueError("Turn in place or move forward; execute them separately")
        self.collection_events = []
        if self.failure:
            return
        stop = self._wall_contact(distance, rotation)
        failure = "Map boundary" if math.isfinite(stop) else None
        circle_exit = self._motion_circle_exit(distance)
        if circle_exit < stop:
            stop, failure = circle_exit, "Motion circle boundary"
        collision_point = None
        obstacle_time, obstacle = self._obstacle_contact(distance, rotation)
        if obstacle_time < stop:
            stop = obstacle_time
            failure = f"Hit obstacle #{obstacle}"
            collision_point = self.obstacles[obstacle - 1][:2]
        collections = []
        groups = tuple((key,) for key in self.targets) if self.pickup_rule == "center" else self.target_groups
        for group in groups:
            contacts = []
            for key in group:
                local = self.local_point(*self.targets[key])
                body_contact = (self._initial_intrusion_contact(*local, distance, rotation)
                                if key in self._initial_intrusions else
                                self.safety_body.first_contact(*local, distance, rotation, self.target_radius))
                contacts.append((body_contact,
                                 (self.collector.first_contact(*local, distance, rotation, self.pickup_radius)
                                  if self.collection_mode == "contact" else math.inf), key))
            body_time, _, hit = min(contacts)
            collect_time = min(contact[1] for contact in contacts)
            if collect_time + EPS < body_time:
                collections.append((collect_time, group))
            elif body_time < stop:
                stop, failure, collision_point = body_time, f"Body hit target #{hit}", self.targets[hit]
        for time, group in sorted(collections):
            if math.isfinite(time) and time <= stop + EPS:
                px, py, yaw = self.pose
                self.collection_events.append({"time": time, "target_ids": group,
                    "pose": (px + distance * time * math.cos(yaw), py + distance * time * math.sin(yaw),
                             normalize(yaw + rotation * time))})
                for key in group:
                    self.collected[key] = self.targets.pop(key)
        t = min(1.0, stop)
        x, y, theta = self.pose
        self.pose = (x + distance * t * math.cos(theta),
                     y + distance * t * math.sin(theta), normalize(theta + rotation * t))
        if self.pose[:2] != self.path[-1]:
            self.path.append(self.pose[:2])
        if self._initial_intrusions:
            self._initial_intrusions = frozenset(
                key for key in self._initial_intrusions if key in self.targets
                and self.safety_body.intersects_circle(*self.local_point(*self.targets[key]), self.target_radius)
            )
        self.failure, self.collision_point = failure, collision_point

    def turn_to(self, degrees: float) -> None:
        self.advance(rotation=normalize(math.radians(finite(degrees)) - self.pose[2]))

    def forward(self, metres: float) -> None:
        self.advance(distance=metres)

    def snapshot(self) -> dict:
        result = {"pose": list(self.pose), "remaining": dict(self.targets),
                "collected": dict(self.collected), "failure": self.failure,
                "scenario": self.scenario, "seed": self.seed,
                "dimensions": self.dimensions, "target_diameter": self.target_diameter}
        if self.obstacles:
            result["obstacles"] = [list(disk) for disk in self.obstacles]
        custom = self.config["custom"]
        if custom.get("fence") is not None:
            result["fence"] = deepcopy(custom["fence"])
        if "fence_enabled" in custom or "fence" in result:
            result["fence_enabled"] = self.fence_enabled
        if self.motion_circle is not None:
            result["motion_circle"] = list(self.motion_circle)
        return result
