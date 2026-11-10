"""Local pickup correction using the latest target observations."""

from dataclasses import replace
import math
import time
from typing import Iterable, Mapping

from route_plan_sim.scene import EPS, normalize

from .api import PickupStop, PlanningRequest, PlanningResponse, PlanStep, TargetId, _empty_scene, _number
from .motion import compact_goals, path_length, replay, steer
from .records import describe_scene

ADJUSTMENT_METHOD = "pickup-adjustment-v1"
DEFAULT_MAX_DISPLACEMENT = .40


def _offsets(lower, upper):
    margin = min(.01, (upper - lower) / 4)
    inner_min, inner_max = lower + margin, upper - margin
    values = {inner_min, (lower + upper) / 2, inner_max, min(inner_max, max(inner_min, 0.))}
    if lower <= 0. <= upper:
        values.add(0.)
    return values


def _candidate_poses(scene, points, limit, deadline):
    x, y, yaw = scene.pose
    offsets = [(tx - x, ty - y) for tx, ty in points]
    bearing = math.atan2(sum(dy for _, dy in offsets), sum(dx for dx, _ in offsets))
    headings = (yaw, bearing, *(normalize(yaw + i * math.tau / 72) for i in range(1, 72)))
    box = scene.collector
    poses = {}
    for angle in headings:
        if time.monotonic() >= deadline:
            break
        c, s = math.cos(angle), math.sin(angle)
        u, v = zip(*((c * dx + s * dy, -s * dx + c * dy) for dx, dy in offsets))
        left, right = max(u) - box.xmax, min(u) - box.xmin
        bottom, top = max(v) - box.ymax, min(v) - box.ymin
        if left > right or bottom > top:
            continue
        for a in _offsets(left, right):
            for b in _offsets(bottom, top):
                distance = math.hypot(a, b)
                if distance > limit + EPS:
                    continue
                dx, dy = c * a - s * b, s * a + c * b
                approach = math.atan2(dy, dx) if distance > EPS else yaw
                turns = abs(normalize(approach - yaw)) + abs(normalize(angle - approach))
                clearance = min(a - left, right - a, b - bottom, top - b)
                poses[x + dx, y + dy, angle] = (distance + .12 * turns, -clearance)
    return sorted(poses, key=lambda pose: (poses[pose], pose))


def _approaches(scene, poses, limit, deadline):
    for goal in poses:
        if time.monotonic() >= deadline:
            return
        transition = steer(scene, goal, minimum_motion=True)
        if transition is not None:
            yield transition.goals

    # If a direct correction fails, try a short pre-aligned approach.
    for goal in poses:
        x, y, yaw = goal
        for distance in (.08, .15, .25):
            if time.monotonic() >= deadline:
                return
            staging = (x - distance * math.cos(yaw), y - distance * math.sin(yaw), yaw)
            if math.dist(staging[:2], scene.pose[:2]) > limit + EPS:
                continue
            first = steer(scene, staging, minimum_motion=True)
            if first is None:
                continue
            last = steer(first.scene, goal, minimum_motion=True)
            if last is not None:
                yield first.goals + last.goals


def plan_pickup_adjustment(
    request: PlanningRequest | Mapping, target_ids: Iterable[TargetId], *,
    max_displacement: float = DEFAULT_MAX_DISPLACEMENT,
) -> PlanningResponse:
    """Find one safe nearby pickup pose covering the requested observed IDs.

    The request pose is measured after arriving at a nominal stop. Include all
    uncollected targets with their latest map-frame positions. The displacement
    limit bounds the vehicle centre relative to that measured pose in metres.
    """
    started = time.monotonic()
    request = request if isinstance(request, PlanningRequest) else PlanningRequest.from_dict(request)
    request = replace(request, method="stops")
    deadline = started + request.timeout
    limit = _number(max_displacement, "max_displacement")
    if limit < 0.:
        raise ValueError("max_displacement must be nonnegative")
    if isinstance(target_ids, (str, bytes)):
        raise ValueError("target_ids must be an iterable of IDs")
    try:
        requested = tuple(target_ids)
    except TypeError as error:
        raise ValueError("target_ids must be an iterable of IDs") from error
    if not requested or any(type(key) not in (str, int) or key == "" for key in requested):
        raise ValueError("target_ids must contain at least one valid ID")
    requested = tuple(dict.fromkeys(requested))

    response = PlanningResponse("not_found", request.frame_id, request.request_id,
                                remaining_ids=[target.id for target in request.targets])

    def finish(status, reason=None):
        response.status, response.reason = status, reason
        response.elapsed = time.monotonic() - started
        return response

    observed = {target.id: (target.x, target.y) for target in request.targets}
    missing = [key for key in requested if key not in observed]
    if missing:
        return finish("target_not_observed", f"Missing updated observations for {missing!r}")
    scene = _empty_scene(request)
    response.resolved_request = request.to_dict()
    response.resolved_request["robot_config"] = describe_scene(scene)["vehicle_config"]
    for key, point in observed.items():
        if scene.safety_body.intersects_circle(*scene.local_point(*point), scene.target_radius):
            return finish("unsafe_start", f"Target {key!r} touches the current safety footprint")
    id_map = dict(zip(scene.add_targets(observed.values()), observed))
    needed = {key for key, name in id_map.items() if name in requested}

    def checked_pickup(goals):
        goals = compact_goals(scene.pose, goals) or [scene.pose]
        try:
            at_stop = replay(scene, goals, dt=.013, minimum_motion=True)
            if at_stop.failure:
                return False
            removed = at_stop.collect_at_stop()
        except ValueError:
            return False
        if not needed.issubset(removed):
            return False
        picked = tuple(id_map[key] for key in removed)
        response.goals = goals
        response.steps = [PlanStep(goal) for goal in goals[:-1]] + [PlanStep(goals[-1], picked)]
        response.pickup_stops = [PickupStop(len(goals) - 1, goals[-1], picked)]
        response.stop_count = 1
        response.expected_collected_ids = list(picked)
        response.remaining_ids = [id_map[key] for key in at_stop.targets]
        response.final_pose = at_stop.pose
        response.length = path_length(scene.pose, goals)
        return True

    if checked_pickup([]):
        return finish("success")
    poses = _candidate_poses(scene, [observed[key] for key in requested], limit, deadline)
    for goals in _approaches(scene, poses, limit, deadline):
        if checked_pickup(goals):
            return finish("success")
    if time.monotonic() >= deadline:
        return finish("budget_exhausted", "Local pickup search budget reached")
    return finish("not_found", "No safe nearby pickup pose covers the requested targets")
