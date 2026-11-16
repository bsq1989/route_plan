"""Local pickup correction using the latest target observations."""

from dataclasses import replace
import math
import time
from typing import Iterable, Mapping

from route_plan_sim.boundary import PolygonFootprint
from route_plan_sim.scene import EPS, normalize

from .api import (DEFAULT_PICKUP_MARGIN_M, PickupStop, PlanningRequest, PlanningResponse, PlanStep, TargetId,
                  _empty_scene, _number, _robot_profile, _workspace_padding)
from .escape import _escape_approaches
from .motion import MIN_TRANSLATION, compact_goals, path_length, replay, steer
from .records import describe_scene

ADJUSTMENT_METHOD = "pickup-adjustment-v1"
DEFAULT_MAX_DISPLACEMENT = .40
SIDE_PICKUP_PENALTY_M = .06


def _translate_pose(pose, source, destination):
    if source == destination:
        return tuple(pose)
    return (pose[0] - source[0] + destination[0], pose[1] - source[1] + destination[1], pose[2])


def _simulation_requests(request, limit):
    """Return the public request and its equivalent bounded simulation request."""
    if isinstance(request, PlanningRequest):
        request = replace(request, method="stops")
        return request, request
    if not isinstance(request, Mapping):
        raise ValueError("Request must be a mapping")
    width, height = request.get("width"), request.get("height")
    if (request.get("mode") == "auto_fence" or request.get("fence") is not None
            or request.get("fence_enabled", True) is not True):
        request = replace(PlanningRequest.from_dict(request), method="stops")
        return request, request
    if (width is None) != (height is None):
        raise ValueError("Provide both width and height, or omit both for local pickup")
    if width is not None:
        request = replace(PlanningRequest.from_dict(request), method="stops")
        return request, request

    # Reuse input validation before deriving the simulator dimensions.
    diameter = _number(request.get("target_diameter", .10), "target_diameter")
    placeholder = max(1., 2 * diameter)
    request = replace(PlanningRequest.from_dict({**request, "width": placeholder, "height": placeholder}),
                      method="stops")
    profile = _robot_profile(request.robot_config)
    padding = _workspace_padding(profile, diameter, limit)
    x, y, yaw = request.pose
    xs = [0., *(target.x - x for target in request.targets)]
    ys = [0., *(target.y - y for target in request.targets)]
    pose = (padding - min(xs), padding - min(ys), yaw)
    targets = [replace(target, x=pose[0] + dx, y=pose[1] + dy)
               for target, dx, dy in zip(request.targets, xs[1:], ys[1:])]
    simulated = replace(request, width=max(xs) - min(xs) + 2 * padding,
                        height=max(ys) - min(ys) + 2 * padding, pose=pose, targets=targets, robot_config=profile)
    return request, simulated


def _offsets(lower, upper):
    margin = min(.01, (upper - lower) / 4)
    inner_min, inner_max = lower + margin, upper - margin
    values = {inner_min, (lower + upper) / 2, inner_max, min(inner_max, max(inner_min, 0.))}
    if lower <= 0. <= upper:
        values.add(0.)
    return values


def _near_edge_clearance(collector, x, y):
    if isinstance(collector, PolygonFootprint):
        return next((max(0., x - left) for left, right in collector.longitudinal_spans(y)
                     if left - EPS <= x <= right + EPS), 0.)
    return max(0., x - collector.xmin)


def _pickup_clearance(collector, x, y):
    if isinstance(collector, PolygonFootprint):
        return min(edge.distance((x, y)) for edge in collector.polygon.edges)
    return min(x - collector.xmin, collector.xmax - x, y - collector.ymin, collector.ymax - y)


def _candidate_poses(scene, points, origin, limit, deadline):
    x, y, yaw = scene.pose
    offsets = [(tx - x, ty - y) for tx, ty in points]
    bearing = math.atan2(sum(dy for _, dy in offsets), sum(dx for dx, _ in offsets))
    headings = (yaw, bearing, *(normalize(yaw + i * math.tau / 72) for i in range(1, 72)))
    box = scene.collector
    margin = scene.pickup_margin_m
    poses = {}
    for angle in headings:
        if time.monotonic() >= deadline:
            break
        c, s = math.cos(angle), math.sin(angle)
        u, v = zip(*((c * dx + s * dy, -s * dx + c * dy) for dx, dy in offsets))
        left, right = max(u) - box.xmax + margin, min(u) - box.xmin - margin
        bottom, top = max(v) - box.ymax + margin, min(v) - box.ymin - margin
        if left > right or bottom > top:
            continue
        offsets_in_heading = [(a, b) for a in _offsets(left, right) for b in _offsets(bottom, top)]
        samples = getattr(box, "pickup_samples", None)
        if samples is not None:
            mid_u, mid_v = (min(u) + max(u)) / 2, (min(v) + max(v)) / 2
            offsets_in_heading.extend((mid_u - a, mid_v - b) for a, b in samples)
        for a, b in offsets_in_heading:
            local_centres = [(p - a, q - b) for p, q in zip(u, v)]
            if (samples is not None or margin) and not all(scene.covers_pickup_center(*point)
                                                          for point in local_centres):
                continue
            distance = math.hypot(a, b)
            dx, dy = c * a - s * b, s * a + c * b
            if math.dist((x + dx, y + dy), origin) > limit + EPS:
                continue
            approach = math.atan2(dy, dx) if distance > EPS else yaw
            turns = abs(normalize(approach - yaw)) + abs(normalize(angle - approach))
            near_clearance = min(_near_edge_clearance(box, *point) for point in local_centres)
            clearance = min(_pickup_clearance(box, *point) for point in local_centres)
            # A bounded preference discourages edge-hugging without making
            # clearance a new pickup constraint or rewarding long detours.
            near_penalty = 2 * max(0., DEFAULT_PICKUP_MARGIN_M - near_clearance)
            edge_penalty = max(0., DEFAULT_PICKUP_MARGIN_M - clearance)
            side_penalty = 0.
            if scene.center_collector is not None:
                side_count = sum(not scene.prefers_pickup_center(*point) for point in local_centres)
                side_penalty = SIDE_PICKUP_PENALTY_M * side_count / len(local_centres)
            cost = distance + .12 * turns + near_penalty + edge_penalty + side_penalty
            poses[x + dx, y + dy, angle] = (cost, -clearance)
    return sorted(poses, key=lambda pose: (poses[pose], pose))


def _approaches(scene, poses, origin, limit, deadline):
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
            if math.dist(staging[:2], origin) > limit + EPS:
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
    uncollected targets in the same fixed coordinate frame. A mapping may omit
    both map dimensions. The displacement limit is relative to the measured pose.
    """
    started = time.monotonic()
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

    request, simulation = _simulation_requests(request, limit)
    deadline = started + request.timeout
    response = PlanningResponse("not_found", request.frame_id, request.request_id,
                                remaining_ids=[target.id for target in request.targets])

    def finish(status, reason=None):
        response.status, response.reason = status, reason
        response.elapsed = time.monotonic() - started
        return response

    observed = {target.id: (target.x, target.y) for target in simulation.targets}
    missing = [key for key in requested if key not in observed]
    if missing:
        return finish("target_not_observed", f"Missing updated observations for {missing!r}")
    scene = _empty_scene(simulation, extra_padding=limit)
    response.resolved_request = request.to_dict()
    if simulation is not request:
        del response.resolved_request["width"], response.resolved_request["height"]
    response.resolved_request["robot_config"] = describe_scene(scene)["vehicle_config"]
    intrusions = set()
    for index, (key, point) in enumerate(observed.items()):
        local = scene.local_point(*point)
        if scene.body.intersects_circle(*local, scene.target_radius):
            return finish("body_contact", f"Target {key!r} touches the physical body")
        if scene.safety_body.intersects_circle(*local, scene.target_radius):
            intrusions.add(index)
    id_map = dict(zip(scene.add_targets(observed.values(), allow_safety_intrusions=bool(intrusions)), observed))
    needed = {key for key, name in id_map.items() if name in requested}

    def checked_pickup(goals, escape_goal_count):
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
        length = path_length(scene.pose, goals)
        goals = [_translate_pose(goal, simulation.pose, request.pose) for goal in goals]
        response.goals = goals
        response.steps = [PlanStep(goal) for goal in goals[:-1]] + [PlanStep(goals[-1], picked)]
        response.pickup_stops = [PickupStop(len(goals) - 1, goals[-1], picked)]
        response.stop_count = 1
        response.expected_collected_ids = list(picked)
        response.remaining_ids = [id_map[key] for key in at_stop.targets]
        response.final_pose = _translate_pose(at_stop.pose, simulation.pose, request.pose)
        response.length = length
        centres = [at_stop.local_point(*observed[key]) for key in requested]
        response.metrics["min_near_edge_clearance_m"] = min(
            _near_edge_clearance(at_stop.collector, *point) for point in centres)
        response.metrics["min_pickup_clearance_m"] = min(
            _pickup_clearance(at_stop.collector, *point) for point in centres)
        if at_stop.center_collector is not None:
            response.metrics["center_pickup_count"] = sum(at_stop.prefers_pickup_center(*point) for point in centres)
        if escape_goal_count:
            response.metrics["escape_goal_count"] = escape_goal_count
        return True

    def adjust_from(start, prefix):
        if checked_pickup(prefix, len(prefix)):
            return True
        origin = scene.pose[:2]
        poses = _candidate_poses(start, [observed[key] for key in requested], origin, limit, deadline)
        return any(checked_pickup([*prefix, *goals], len(prefix))
                   for goals in _approaches(start, poses, origin, limit, deadline))

    if intrusions:
        found_escape = False
        prefixes = _escape_approaches(scene, list(observed.values()), intrusions, deadline=deadline,
                                      minimum_translation=MIN_TRANSLATION, max_displacement=limit)
        for prefix in prefixes:
            try:
                escaped = replay(scene, prefix, dt=.013, minimum_motion=True)
            except ValueError:
                continue
            if escaped.failure or escaped.recovering:
                continue
            found_escape = True
            if adjust_from(escaped, prefix):
                return finish("success")
        if not found_escape and time.monotonic() < deadline:
            return finish("not_found", "No safe escape inside the local displacement limit")
    elif adjust_from(scene, []):
        return finish("success")
    if time.monotonic() >= deadline:
        return finish("budget_exhausted", "Local pickup search budget reached")
    return finish("not_found", "No safe nearby pickup pose covers the requested targets")
