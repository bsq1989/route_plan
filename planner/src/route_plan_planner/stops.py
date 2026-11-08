"""Budgeted lexicographic minimization of pickup stops, then travel distance."""

from copy import deepcopy
from dataclasses import dataclass
import math
import time

from route_plan_sim import Scene
from route_plan_sim.scene import EPS, normalize

from .core import PlannerOptions, PlanResult, StaticPlanner
from .motion import DirectMotion, compact_goals, execute, path_length, replay


def at_stop_scene(scene: Scene) -> Scene:
    result = scene.clone()
    result.collection_mode = "at_stop"
    result.config["custom"]["collection_mode"] = "at_stop"
    result.pickup_rule = "center"
    result.config["custom"]["pickup_rule"] = "center"
    return result


def center_contact_scene(scene: Scene) -> Scene:
    """Feasibility seed with centre-entry events, preserving disk collisions."""
    result = at_stop_scene(scene)
    result.collection_mode = "contact"
    result.config["custom"]["collection_mode"] = "contact"
    result._collect_at_pose()
    return result


def materialize_stops(scene: Scene, initial: PlanResult) -> PlanResult:
    """Split a centre-entry plan at exact analytic entry poses, never at frames.

    This supplies a feasible incumbent under explicit pickup semantics. Every
    individual contact becomes a real stop unless several disks are covered
    simultaneously at that pose. Merely crossing several disks is not one stop.
    """
    if initial.status != "success":
        return deepcopy(initial)
    trial = at_stop_scene(scene)
    contact = center_contact_scene(scene)
    goals, stops = [], []

    def append_goal():
        pose = tuple(trial.pose)
        if not goals or math.dist(pose[:2], goals[-1][:2]) > EPS or abs(normalize(pose[2] - goals[-1][2])) > EPS:
            goals.append(pose)
        return len(goals) - 1

    def pickup():
        removed = trial.collect_at_stop()
        if removed:
            index = append_goal()
            stops.append({"goal_index": index, "pose": tuple(goals[index]), "target_ids": list(removed)})

    if set(contact.collected) - set(trial.collected):
        pickup()
    for goal in initial.goals:
        dx, dy = goal[0] - contact.pose[0], goal[1] - contact.pose[1]
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if distance > EPS else contact.pose[2]
        phases = (("rotation", normalize(bearing - contact.pose[2])), ("distance", distance),
                  ("rotation", normalize(goal[2] - bearing)))
        for kind, amount in phases:
            if abs(amount) <= EPS:
                continue
            contact.advance(**{kind: amount})
            if contact.failure:
                raise ValueError("Initial contact plan is not feasible")
            last_time = 0.
            for event in contact.collection_events:
                if not set(event["target_ids"]) & trial.targets.keys():
                    continue
                progress = max(last_time, event["time"])
                trial.advance(**{kind: amount * (progress - last_time)})
                if trial.failure:
                    raise ValueError("Contact plan cannot be split into safe stops")
                pickup()
                if set(event["target_ids"]) & trial.targets.keys():
                    raise ValueError("An analytic contact is not covered at its stop")
                last_time = progress
            trial.advance(**{kind: amount * (1. - last_time)})
            if trial.failure:
                raise ValueError("Split contact plan collides after pickup")
            append_goal()
    final = replay(at_stop_scene(scene), goals, pickup_stops=stops)
    if final.failure or final.targets:
        raise ValueError("Stop incumbent failed independent replay")
    result = deepcopy(initial)
    result.goals, result.pickup_stops = goals, stops
    result.length = path_length(scene.pose, goals)
    result.final = final.snapshot()
    result.remaining = result.best_remaining = 0
    return result


@dataclass(frozen=True)
class _Candidate:
    pose: tuple[float, float, float]
    covered: frozenset[int]
    blocked: frozenset[int]


class StopsPlanner:
    """Finite candidate coverage search; no global optimum is claimed."""

    def __init__(self, options: PlannerOptions | None = None):
        self.options = options or PlannerOptions()

    def _pool(self, scene, deadline, incumbent):
        points = list(scene.targets.values())
        anchors = set(points)
        diameter = math.hypot(scene.dimensions["collector_depth"], scene.dimensions["collector_width"])
        for i, point in enumerate(points):
            for other in points[:i]:
                if math.dist(point, other) <= diameter:
                    anchors.add(((point[0] + other[0]) / 2, (point[1] + other[1]) / 2))
        near = max(scene.collector.xmin, scene.safety_body.xmax + scene.target_radius) + .015
        far = scene.collector.xmax - .015
        depths = sorted({far, (near + far) / 2, near})
        offset = (scene.collector.ymin + scene.collector.ymax) / 2
        poses = {tuple(stop["pose"]) for stop in incumbent.pickup_stops}
        poses.add(tuple(scene.pose))
        for tx, ty in sorted(anchors):
            angles = [scene.pose[2], math.atan2(ty - scene.pose[1], tx - scene.pose[0])]
            angles += [i * math.tau / self.options.headings for i in range(self.options.headings)]
            for angle in angles:
                c, s = math.cos(angle), math.sin(angle)
                for depth in depths:
                    poses.add((tx - c * depth + s * offset, ty - s * depth - c * offset, normalize(angle)))
        candidates = []
        for pose in sorted(poses):
            if time.monotonic() >= deadline:
                break
            x, y, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            # Check both full footprints against the wall, including rotations
            # later in the connector; this is only the cheap endpoint filter.
            if any(not (EPS < x + c * u - s * v < scene.width - EPS
                        and EPS < y + s * u + c * v < scene.height - EPS)
                   for rect in (scene.safety_body, scene.collector) for u, v in rect.corners):
                continue
            covered, blocked = set(), set()
            for key, (tx, ty) in scene.targets.items():
                u, v = c * (tx - x) + s * (ty - y), -s * (tx - x) + c * (ty - y)
                if scene.collector.contains(u, v):
                    covered.add(key)
                if scene.safety_body.intersects_circle(u, v, scene.target_radius):
                    blocked.add(key)
            if covered:
                candidates.append(_Candidate(pose, frozenset(covered), frozenset(blocked)))
        return candidates

    def plan(self, scene: Scene, initial: PlanResult | None = None) -> PlanResult:
        started = time.monotonic()
        shared_elapsed = initial.elapsed if initial is not None else 0.
        deadline = started + max(0., self.options.timeout - shared_elapsed)
        root = at_stop_scene(scene)
        if initial is None:
            contact = center_contact_scene(scene)
            initial = StaticPlanner(self.options).plan(contact)
        if initial.status != "success":
            return deepcopy(initial)
        try:
            best = materialize_stops(root, initial)
        except ValueError as error:
            return PlanResult("replay_failed", reason=str(error), remaining=len(root.targets))
        baseline_count, baseline_length = len(best.pickup_stops), best.length
        stats = {"method": "stop-cover-beam-v1", "objective": ["pickup_stop_count", "translation_length_m"],
                 "pickup_rule": "center",
                 "baseline_stop_count": baseline_count, "baseline_length": baseline_length,
                 "stop_count": baseline_count, "improvements": 0, "global_optimal": False,
                 "candidate_count": 0, "trials": 0, "expanded": 0, "termination": "candidates_exhausted"}
        best.optimization = stats
        pool_deadline = min(deadline, time.monotonic() + max(0., deadline - time.monotonic()) * .3)
        pool = self._pool(root, pool_deadline, best)
        stats["candidate_count"] = len(pool)
        connector = DirectMotion()
        fallback = None
        if self.options.ompl:
            from .ompl_motion import OmplMotion
            fallback = OmplMotion(min(.15, self.options.ompl_seconds), self.options.seed)
        layer = [(root, [], [], 0.)]
        seen = {}
        trial_budget = max(0, self.options.max_trials - initial.trials)
        expansion_budget = max(0, self.options.max_expansions - initial.expanded)
        while (layer and time.monotonic() < deadline and stats["trials"] < trial_budget
               and stats["expanded"] < expansion_budget):
            next_layer = []
            for current, goals, stops, cost in layer:
                if time.monotonic() >= deadline or stats["expanded"] >= expansion_budget:
                    break
                if len(stops) >= len(best.pickup_stops):
                    continue
                stats["expanded"] += 1
                remaining = frozenset(current.targets)
                choices = {}
                for candidate in pool:
                    coverage = candidate.covered & remaining
                    if not coverage or candidate.blocked & remaining:
                        continue
                    distance = math.dist(current.pose[:2], candidate.pose[:2])
                    bucket = choices.setdefault(coverage, [])
                    bucket.append((distance, candidate.pose))
                proposals = []
                for coverage, values in choices.items():
                    for distance, pose in sorted(values)[:3]:
                        proposals.append((-len(coverage), distance, pose))
                proposals.sort()
                accepted = 0
                for _, _, pose in proposals[:64]:
                    if time.monotonic() >= deadline or stats["trials"] >= trial_budget:
                        break
                    stats["trials"] += 1
                    transition = connector.connect(current, pose, deadline)
                    if transition is None and fallback and accepted == 0 and stats["trials"] % 24 == 0:
                        transition = fallback.connect(current, pose, deadline)
                    if transition is None:
                        continue
                    child = transition.scene
                    removed = child.collect_at_stop()
                    if not removed:
                        continue
                    segment = compact_goals(current.pose, transition.goals)
                    if not segment:
                        segment = [tuple(child.pose)]
                    child_goals = goals + segment
                    child_stops = stops + [{"goal_index": len(child_goals) - 1, "pose": tuple(child_goals[-1]),
                                            "target_ids": list(removed)}]
                    child_cost = cost + path_length(current.pose, segment)
                    if not child.targets:
                        objective = (len(child_stops), child_cost)
                        incumbent = (len(best.pickup_stops), best.length)
                        if objective[0] < incumbent[0] or objective[0] == incumbent[0] and objective[1] < incumbent[1] - 1e-8:
                            checked = replay(root, child_goals, pickup_stops=child_stops)
                            if not checked.failure and not checked.targets:
                                best.goals, best.pickup_stops, best.length = child_goals, child_stops, child_cost
                                best.final = checked.snapshot()
                                stats["improvements"] += 1
                        continue
                    key = StaticPlanner._key(child, 0)
                    value = (len(child_stops), child_cost)
                    if key in seen and seen[key] <= value:
                        continue
                    seen[key] = value
                    maximum = max((len(candidate.covered & child.targets.keys()) for candidate in pool), default=1)
                    estimate = len(child_stops) + math.ceil(len(child.targets) / max(1, maximum))
                    next_layer.append((estimate, len(child.targets), child_cost, child, child_goals, child_stops))
                    accepted += 1
                    if accepted >= 4:
                        break
            next_layer.sort(key=lambda item: item[:3])
            layer = [(child, goals, stops, cost) for _, _, cost, child, goals, stops in next_layer[:4]]
        if time.monotonic() >= deadline:
            stats["termination"] = "budget_exhausted_kept_incumbent"
        checked = replay(root, best.goals, dt=.013, pickup_stops=best.pickup_stops)
        if checked.failure or checked.targets:
            return PlanResult("replay_failed", reason="Final pickup-stop replay failed", remaining=len(checked.targets))
        best.final = checked.snapshot()
        best.remaining = best.best_remaining = 0
        best.length = path_length(root.pose, best.goals)
        best.elapsed = shared_elapsed + time.monotonic() - started
        best.trials = initial.trials + stats["trials"]
        best.expanded = initial.expanded + stats["expanded"]
        stats["stop_count"] = len(best.pickup_stops)
        stats["saved_stops"] = baseline_count - len(best.pickup_stops)
        stats["saved_length"] = baseline_length - best.length
        return best
