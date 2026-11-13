"""Replan one observation leg or pickup stop from the latest map observations."""

from collections import Counter
from dataclasses import asdict, dataclass, field, replace
import math
from typing import Mapping

from route_plan_sim.boundary import Fence
from route_plan_sim.scene import EPS, Rect, normalize

from .api import PickupStop, PlanningRequest, Pose, Target, _empty_scene, _number, plan
from .motion import MIN_ROTATION, MIN_TRANSLATION

_HEADINGS = tuple(index * math.tau / 24 for index in range(24))
_DIRECT_STEPS = (.075, .12, .20, .30, .40, .55, .70, .90, 1.15)
_HOP_STEPS = (.075, .16, .28, .46, .70, .95)


@dataclass(frozen=True)
class CoarseTarget(Target):
    """An unconfirmed target whose centre lies in a bounded error disk."""

    error_radius: float

    def __post_init__(self):
        super().__post_init__()
        error = _number(self.error_radius, "coarse target error_radius")
        if error <= 0.:
            raise ValueError("Coarse target error_radius must be positive")
        object.__setattr__(self, "error_radius", error)


@dataclass
class ObservationResponse:
    status: str
    frame_id: str
    request_id: str | None
    goals: list[Pose] = field(default_factory=list)
    pickup_stops: list[PickupStop] = field(default_factory=list)
    observation_pose: Pose | None = None
    pending_hops: list[list[Pose]] = field(default_factory=list)
    observations_since_pickup: int = 0
    reason: str | None = None
    rejections: dict[str, int] = field(default_factory=dict)

    @property
    def success(self):
        return self.status in ("observe", "pickup")

    def to_dict(self):
        result = asdict(self)
        result["success"] = self.success
        result["goals"] = [list(goal) for goal in self.goals]
        result["pickup_stops"] = [
            {"goal_index": stop.goal_index, "pose": list(stop.pose), "target_ids": list(stop.target_ids)}
            for stop in self.pickup_stops
        ]
        result["observation_pose"] = list(self.observation_pose) if self.observation_pose is not None else None
        result["pending_hops"] = [[list(goal) for goal in hop] for hop in self.pending_hops]
        result["rejections"] = dict(self.rejections)
        return result


def _pose(value):
    try:
        x, y, yaw = (_number(v, "pose component") for v in value)
    except (TypeError, ValueError) as error:
        raise ValueError("Each pose must contain finite x, y and yaw") from error
    return x, y, normalize(yaw)


def _local(pose, target):
    dx, dy = target.x - pose[0], target.y - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


def _new_sweep_clear(body, x, y, radius, error, distance):
    # The starting footprint is independently certified clear. Bound the
    # newly swept area of its radius-offset front by horizontal slabs.
    if x + error <= body.xmax - EPS:
        return True
    front = body.xmax + radius
    slabs = [(front, front, body.ymin, body.ymax)]
    for index in range(6):
        low, high = radius * index / 6, radius * (index + 1) / 6
        inner = body.xmax + math.sqrt(max(0., radius * radius - high * high))
        outer = body.xmax + math.sqrt(max(0., radius * radius - low * low))
        slabs.extend(((inner, outer, body.ymin - high, body.ymin - low),
                      (inner, outer, body.ymax + low, body.ymax + high)))
    return all(not Rect(a, low, b + distance, high).intersects_circle(x, y, error)
               for a, b, low, high in slabs)


def _motion_parts(start, goal):
    distance = math.dist(start[:2], goal[:2])
    bearing = math.atan2(goal[1] - start[1], goal[0] - start[0]) if distance > EPS else start[2]
    first = normalize(bearing - start[2])
    last = normalize(goal[2] - bearing)
    if abs(first) > EPS:
        yield start, 0., first
    if distance > EPS:
        yield (*start[:2], bearing), distance, 0.
    if abs(last) > EPS:
        yield (*goal[:2], bearing), 0., last


def _goals_for(start, xy, yaw):
    distance = math.dist(start[:2], xy)
    if EPS < distance <= MIN_TRANSLATION + EPS:
        return None
    bearing = math.atan2(xy[1] - start[1], xy[0] - start[0]) if distance > EPS else start[2]
    goals = []
    for origin, end, point in ((start[2], bearing, start[:2]), (bearing, yaw, xy)):
        turn = normalize(end - origin)
        if EPS < abs(turn) <= MIN_ROTATION + EPS:
            return None
        pieces = math.ceil(abs(turn) / (math.pi / 2))
        if abs(turn) > EPS:
            goals.extend((*point, normalize(origin + turn * i / pieces)) for i in range(1, pieces + 1))
        if point == start[:2] and distance > EPS:
            goals.append((*xy, bearing))
    return goals


class _Search:
    def __init__(self, scene, targets, focus, visited, clearance, reliable_range, half_fov):
        self.scene = scene
        self.targets = targets
        self.focus = focus
        self.visited = visited
        self.clearance = clearance
        self.reliable_range = reliable_range
        self.half_fov = half_fov
        xmin, ymin, xmax, ymax = scene.map_bounds
        self.bounds = Fence.from_points(((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)))
        self.rejections = Counter()

    def check(self, start, goals):
        body, collector, radius = self.scene.safety_body, self.scene.collector, self.scene.target_radius
        for goal in goals:
            dx, dy = goal[0] - start[0], goal[1] - start[1]
            if dx * math.cos(start[2]) + dy * math.sin(start[2]) < -EPS:
                return "behind_robot"
            for pose, distance, turn in list(_motion_parts(start, goal)) or [(start, 0., 0.)]:
                if (EPS < distance <= MIN_TRANSLATION + EPS
                        or EPS < abs(turn) <= MIN_ROTATION + EPS):
                    return "motion_deadband"
                for boundary, reason in ((self.bounds, "map_boundary"), (self.scene.fence, "fence")):
                    if boundary is not None and any(math.isfinite(boundary.first_contact(rect, pose, distance, turn))
                                                    for rect in (body, collector)):
                        return reason
                for target, error in self.targets:
                    x, y = _local(pose, target)
                    if not math.isfinite(body.first_contact(x, y, distance, turn, radius + error)):
                        continue
                    if self.clearance and error and not turn and (
                        not distance or _new_sweep_clear(body, x, y, radius, error, distance)
                    ):
                        continue
                    return "uncertain_clearance" if error else "collision_zone"
            start = goal
        return None

    def _visibility(self, pose, target, error):
        x, y = _local(pose, target)
        distance = math.hypot(x, y)
        if (distance <= EPS or x * math.sin(self.half_fov) - abs(y) * math.cos(self.half_fov) < error
                or distance + error > self.reliable_range):
            return False
        ux, uy = x / distance, y / distance
        radius = self.scene.target_radius
        for other, other_error in self.focus:
            if other.id == target.id:
                continue
            ox, oy = _local(pose, other)
            along = min(max(ox * ux + oy * uy, 0.), max(0., distance - radius))
            if math.hypot(ox - ux * along, oy - uy * along) <= radius + error + other_error:
                return False
        return True

    def direct(self):
        start = self.scene.pose
        x, y, yaw = start
        bearings = [math.atan2(t.y - y, t.x - x) for t, _ in self.focus]
        locations = {(x, y)}
        for angle in (*_HEADINGS, *bearings):
            locations.update((x + d * math.cos(angle), y + d * math.sin(angle)) for d in _DIRECT_STEPS)
        target, error = min(self.focus, key=lambda item: math.dist(start[:2], (item[0].x, item[0].y)))
        candidates = []
        for xy in sorted(locations):
            directions = [math.atan2(t.y - xy[1], t.x - xy[0]) for t, _ in self.focus]
            midpoints = [normalize(a + normalize(b - a) / 2)
                         for i, a in enumerate(directions) for b in directions[i + 1:]
                         if abs(normalize(b - a)) <= 2 * self.half_fov]
            travel = math.dist(start[:2], xy)
            bearing = math.atan2(xy[1] - y, xy[0] - x) if travel > EPS else yaw
            remaining = max(0., math.dist(xy, (target.x, target.y)) + error - self.reliable_range)
            face = math.atan2(target.y - xy[1], target.x - xy[0])
            for heading in sorted({normalize(a) for a in (*_HEADINGS, yaw, *directions, *midpoints)}):
                if any(math.dist(xy, p[:2]) < .04 and abs(normalize(heading - p[2])) < .04 for p in self.visited):
                    continue
                goals = _goals_for(start, xy, heading)
                if not goals:
                    continue
                pose = (*xy, heading)
                gain = self._visibility(pose, target, error)
                cost = travel + .12 * (abs(normalize(bearing - yaw)) + abs(normalize(heading - bearing)))
                candidates.append(((-gain, cost if gain else remaining, abs(normalize(heading - face)), cost, pose), goals))
        for _, goals in sorted(candidates):
            reason = self.check(start, goals)
            if not reason:
                return [goals]
            self.rejections["observe_" + reason] += 1
        for distance in reversed(_DIRECT_STEPS):
            goal = (x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)
            if any(math.dist(goal[:2], p[:2]) < .04 for p in self.visited):
                continue
            reason = self.check(start, [goal])
            if not reason:
                return [[goal]]
            self.rejections["observe_" + reason] += 1
        return []

    def coverage(self, pose):
        gain, closest = 0., math.inf
        for target, error in self.focus:
            x, y = _local(pose, target)
            distance = math.hypot(x, y)
            angle = abs(math.atan2(y, x))
            closest = min(closest, max(0., distance + error - self.reliable_range) + .15 * angle)
            margin = min(self.reliable_range - distance, distance * math.sin(self.half_fov - angle))
            if margin >= error:
                gain += 1.
            elif error and margin > -error and angle < math.pi / 2:
                gain += .5 * (1 + margin / error)
        return gain, closest

    def detour(self):
        def key(pose):
            return round(pose[0] / .07), round(pose[1] / .07), round(normalize(pose[2]) / .12)

        used = {key(pose) for pose in self.visited}
        explored = {key(self.scene.pose)}
        frontier = [(self.scene.pose, [], 0.)]
        best = None
        for _ in range(3):
            children = []
            for start, path, cost in frontier:
                bearings = [math.atan2(t.y - start[1], t.x - start[0]) for t, _ in self.focus]
                headings = sorted({round(normalize(a), 10) for a in (*_HEADINGS[::2], *bearings, start[2])})
                for heading in headings:
                    if abs(normalize(heading - start[2])) < EPS:
                        heading = start[2]
                    turn = _goals_for(start, start[:2], heading)
                    if turn is None or self.check(start, turn):
                        continue
                    rotated = (*start[:2], heading)
                    cost_with_turn = cost + .12 * abs(normalize(heading - start[2]))
                    for distance in _HOP_STEPS:
                        pose = (start[0] + distance * math.cos(heading),
                                start[1] + distance * math.sin(heading), heading)
                        cell = key(pose)
                        if cell in explored or self.check(rotated, [pose]):
                            continue
                        explored.add(cell)
                        hop = [*turn, pose]
                        new_path, new_cost = [*path, hop], cost_with_turn + distance
                        gain, closest = self.coverage(pose)
                        if cell not in used:
                            rank = (-gain, closest if not gain else 0., new_cost, pose)
                            children.append((rank, pose, new_path, new_cost))
                        faces = {heading, *(math.atan2(t.y - pose[1], t.x - pose[0]) for t, _ in self.focus)}
                        for face in sorted(faces):
                            view = (*pose[:2], face)
                            if key(view) in used:
                                continue
                            gain, closest = self.coverage(view)
                            rank = (-gain, closest if not gain else 0.,
                                    new_cost + .12 * abs(normalize(face - heading)), view)
                            if best is not None and rank >= best[0]:
                                continue
                            final = _goals_for(pose, pose[:2], face)
                            if final is not None and not self.check(pose, final):
                                best = rank, [*path, [*hop, *final]]
            frontier = [(pose, path, cost) for _, pose, path, cost in sorted(children)[:8]]
            if not frontier:
                break
        return best[1] if best else []

    def pickup(self, request):
        known = request.targets
        singles = sorted(known, key=lambda t: math.dist(self.scene.pose[:2], (t.x, t.y)))
        batches = [known, *((t,) for t in singles)] if len(known) > 1 else [known]
        for targets in batches:
            if not targets:
                break
            candidate = replace(request, targets=targets, method="stops",
                                timeout=min(request.timeout, .75 if len(targets) > 1 else .25))
            try:
                result = plan(candidate)
            except ValueError:
                self.rejections["pickup_input_rejected"] += 1
                continue
            if not result.success:
                self.rejections["pickup_" + result.status] += 1
                continue
            stop = result.pickup_stops[0]
            goals = result.goals[:stop.goal_index + 1]
            reason = self.check(self.scene.pose, goals)
            if reason:
                self.rejections["pickup_" + reason] += 1
                continue
            ids = {t.id for t in targets}
            if any(self.scene.collector.intersects_circle(*_local(stop.pose, t), error)
                   for t, error in self.targets if t.id not in ids):
                self.rejections["pickup_unconfirmed_at_collector"] += 1
                continue
            return goals, [stop]
        return None


def plan_observation_step(
    request: PlanningRequest | Mapping,
    coarse_targets=(),
    *,
    visited=(),
    pending_hops=(),
    observations_since_pickup=0,
    initial_clearance_confirmed=False,
    reliable_range=1.4,
    horizontal_fov_deg=60.,
) -> ObservationResponse:
    """Return one executable batch. Remeasure and call again before any later hop.

    Exact confirmed map targets go in request.targets. Coarse targets constrain
    motion and rank views. A matching exact ID supersedes its coarse estimate.
    """
    request = request if isinstance(request, PlanningRequest) else PlanningRequest.from_dict(request)
    if type(initial_clearance_confirmed) is not bool:
        raise ValueError("initial_clearance_confirmed must be boolean")
    if type(observations_since_pickup) is not int or observations_since_pickup < 0:
        raise ValueError("observations_since_pickup must be a nonnegative integer")
    reliable_range = _number(reliable_range, "reliable_range")
    fov = _number(horizontal_fov_deg, "horizontal_fov_deg")
    if reliable_range <= 0 or not 0 < fov <= 180:
        raise ValueError("reliable_range must be positive and horizontal_fov_deg must be in (0, 180]")
    try:
        coarse = [item if isinstance(item, CoarseTarget) else CoarseTarget(**item) for item in coarse_targets]
        history = [_pose(value) for value in visited]
        pending = [[_pose(value) for value in hop] for hop in pending_hops]
    except TypeError as error:
        raise ValueError("Invalid coarse_targets, visited or pending_hops") from error
    if len({target.id for target in coarse}) != len(coarse) or any(not hop for hop in pending):
        raise ValueError("Coarse target IDs must be unique and pending hops must be nonempty")
    exact = {target.id: target for target in request.targets}
    coarse_ids = {target.id for target in coarse}
    # Preserve observation order when a coarse target becomes exact.
    targets = [(exact.get(t.id, t), 0. if t.id in exact else t.error_radius) for t in coarse]
    targets.extend((target, 0.) for target in request.targets if target.id not in coarse_ids)
    result = ObservationResponse("no_targets", request.frame_id, request.request_id)
    if not targets:
        return result

    scene = _empty_scene(request, extra_padding=3.)
    focus = [(target, error) for target, error in targets if error] or targets
    search = _Search(scene, targets, focus, [*history, scene.pose], initial_clearance_confirmed,
                     reliable_range, math.radians(fov / 2))
    result.rejections = search.rejections
    pickup = search.pickup(request)
    if pickup:
        result.status = "pickup"
        result.goals, result.pickup_stops = pickup
        return result

    pose = scene.pose
    count = observations_since_pickup + 1
    if pending:
        for hop in pending:
            reason = search.check(pose, hop)
            if reason:
                search.rejections["pending_" + reason] += 1
                pending = []
                break
            pose = hop[-1]
    if not pending:
        pending = search.direct()
        if not pending or count % 3 == 1:
            detour = search.detour()
            if detour and (not pending or search.coverage(detour[-1][-1])[0] > search.coverage(pending[0][-1])[0] + .2
                           or count >= 4 and len(detour) > 1):
                pending = detour
    if not pending:
        result.status = "blocked"
        result.reason = "No observation leg passed the motion and clearance checks"
        return result
    result.status = "observe"
    result.observations_since_pickup = count
    result.observation_pose = pending[-1][-1]
    result.goals, *result.pending_hops = pending
    return result
