"""Budgeted minimization of executable pickup stops."""

from copy import deepcopy
from dataclasses import dataclass, replace
import math
import time

from route_plan_sim import Scene
from route_plan_sim.scene import EPS, normalize

from .core import PlannerOptions, PlanResult, StaticPlanner
from .motion import MIN_ROTATION, MIN_TRANSLATION, DirectMotion, compact_goals, execute, path_length, replay, steer

STOPS_METHOD = "stop-cover-beam-v2"


def _connect_proposals(scene, proposals, connector, fallback, deadline, stats, trial_budget):
    found = False
    for motion, poses in ((connector, proposals), (fallback, proposals[:2])):
        if found:
            return
        if motion is None:
            continue
        for pose in poses:
            if time.monotonic() >= deadline or stats["trials"] >= trial_budget:
                return
            stats["trials"] += 1
            if motion is fallback:
                stats["ompl_calls"] += 1
            transition = motion.connect(scene, pose, deadline)
            if transition is not None:
                if motion is fallback:
                    stats["ompl_connections"] += 1
                found = True
                yield transition


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


def _past_deadband(start, goal):
    """Shift a short move or turn just beyond the deadband for a coverage retry."""
    x, y, yaw = goal
    distance = math.dist(start[:2], goal[:2])
    if EPS < distance <= MIN_TRANSLATION + EPS:
        scale = 1.2 * MIN_TRANSLATION / distance
        return start[0] + scale * (x - start[0]), start[1] + scale * (y - start[1]), yaw
    rotation = normalize(yaw - start[2])
    if distance <= EPS and EPS < abs(rotation) <= MIN_ROTATION + EPS:
        return start[0], start[1], normalize(start[2] + math.copysign(1.2 * MIN_ROTATION, rotation))
    return None


def _executable_seed(root, seed, connector, deadline):
    """Keep the contact-derived pickup poses, reconnecting invalid motion."""
    current = root.clone()
    goals, stops = [], []
    start = 0
    for stop in seed.pickup_stops:
        end = stop["goal_index"] + 1
        segment = seed.goals[start:end]
        start = end
        trial = current.clone()
        try:
            execute(trial, segment, minimum_motion=True)
        except ValueError:
            trial = None
        expected = set(stop["target_ids"])
        if trial is None or trial.failure:
            goal = segment[-1]
            shifted = _past_deadband(current.pose, goal)
            transition = steer(current, shifted, minimum_motion=True) if shifted else None
            if transition is None or set(transition.scene.clone().collect_at_stop()) != expected:
                transition = connector.connect(current, goal, deadline)
            if transition is None:
                # Preserve usable intermediate poses from the contact plan.
                for pose in segment[:-1]:
                    intermediate = connector.connect(current, pose, deadline)
                    if intermediate is not None:
                        goals.extend(compact_goals(current.pose, intermediate.goals))
                        current = intermediate.scene
                transition = connector.connect(current, goal, deadline)
                if transition is None:
                    return None
            trial = transition.scene
            segment = compact_goals(current.pose, transition.goals) or [tuple(trial.pose)]
        if set(trial.collect_at_stop()) != expected:
            return None
        current = trial
        goals.extend(segment)
        stops.append({**stop, "goal_index": len(goals) - 1, "pose": tuple(goals[-1])})
    if current.targets:
        return None
    result = deepcopy(seed)
    result.goals, result.pickup_stops = goals, stops
    result.length = path_length(root.pose, goals)
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
        poses = {tuple(stop["pose"]) for stop in incumbent.pickup_stops} if incumbent else set()
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
        if scene.recovering:
            return self._plan_after_escape(scene, initial, started, shared_elapsed, deadline)
        root = at_stop_scene(scene)
        if initial is None:
            contact = center_contact_scene(scene)
            # A contact solution is optional. Keep the seed cheap and reserve
            # time, trials and OMPL detours for executable stops.
            seed_options = replace(self.options, timeout=self.options.timeout / 3 or self.options.timeout,
                                   max_trials=max(1, self.options.max_trials // 3),
                                   max_expansions=max(1, self.options.max_expansions // 3), ompl=False)
            initial = StaticPlanner(seed_options).plan(contact)
        connector = DirectMotion(minimum_motion=True)
        seed = best = None
        if initial.status == "success":
            try:
                seed = materialize_stops(root, initial)
            except ValueError:
                pass
            if seed is not None:
                best = _executable_seed(root, seed, connector, deadline)
        baseline_count = len(best.pickup_stops) if best else None
        baseline_length = best.length if best else None
        stats = {"method": STOPS_METHOD, "objective": ["pickup_stop_count"],
                 "pickup_rule": "center",
                 "seed_status": initial.status, "seed_elapsed": initial.elapsed,
                 "baseline_stop_count": baseline_count, "baseline_length": baseline_length,
                 "stop_count": baseline_count, "improvements": 0, "global_optimal": False,
                 "candidate_count": 0, "trials": 0, "expanded": 0, "ompl_calls": 0, "ompl_connections": 0,
                 "termination": "candidates_exhausted"}
        pool_deadline = min(deadline, time.monotonic() + max(0., deadline - time.monotonic()) * .3)
        pool = self._pool(root, pool_deadline, seed)
        stats["candidate_count"] = len(pool)
        # With no contact route to supply pickup poses, retain more approaches.
        wide_search = seed is None
        beam_width, approaches_per_batch, proposal_limit = (8, 20, 260) if wide_search else (4, 3, 64)
        fallback = None
        if self.options.ompl:
            from .ompl_motion import OmplMotion
            fallback = OmplMotion(min(.15, self.options.ompl_seconds), self.options.seed, minimum_motion=True)
        layer = [(root, [], [])]
        seen = {}
        ompl_only = False
        trial_budget = max(0, self.options.max_trials - initial.trials)
        expansion_budget = max(0, self.options.max_expansions - initial.expanded)
        while (layer and time.monotonic() < deadline and stats["trials"] < trial_budget
               and stats["expanded"] < expansion_budget):
            next_layer = []
            # When there is no seed, try the whole layer with direct/staged
            # approaches before spending time on random detours.
            direct = None if ompl_only else connector
            local_fallback = fallback if ompl_only or not wide_search else None
            for current, goals, stops in layer:
                if time.monotonic() >= deadline or stats["expanded"] >= expansion_budget:
                    break
                if best is not None and len(stops) + 1 >= len(best.pickup_stops):
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
                    for distance, pose in sorted(values)[:approaches_per_batch]:
                        proposals.append((-len(coverage), distance, pose))
                proposals.sort()
                accepted = 0
                poses = [pose for _, _, pose in proposals[:proposal_limit]]
                for transition in _connect_proposals(current, poses, direct, local_fallback, deadline, stats, trial_budget):
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
                    if not child.targets:
                        if best is None or len(child_stops) < len(best.pickup_stops):
                            try:
                                checked = replay(root, child_goals, pickup_stops=child_stops, minimum_motion=True)
                            except ValueError:
                                continue
                            if not checked.failure and not checked.targets:
                                best = deepcopy(initial)
                                best.status, best.reason = "success", None
                                best.diagnostic = None
                                best.goals, best.pickup_stops = child_goals, child_stops
                                stats["improvements"] += 1
                        continue
                    key = StaticPlanner._key(child, 0)
                    value = len(child_stops)
                    if key in seen and seen[key] <= value:
                        continue
                    seen[key] = value
                    maximum = max((len(candidate.covered & child.targets.keys()) for candidate in pool), default=1)
                    estimate = len(child_stops) + math.ceil(len(child.targets) / max(1, maximum))
                    next_layer.append((estimate, len(child.targets), child, child_goals, child_stops))
                    accepted += 1
                    if accepted >= beam_width:
                        break
            if wide_search and not next_layer and best is None and fallback is not None and not ompl_only:
                ompl_only = True
                continue
            ompl_only = False
            next_layer.sort(key=lambda item: item[:2])
            layer = [(child, goals, stops) for _, _, child, goals, stops in next_layer[:beam_width]]
        if best is None:
            exhausted = (time.monotonic() >= deadline or stats["trials"] >= trial_budget
                         or stats["expanded"] >= expansion_budget)
            status = "budget_exhausted" if exhausted else "not_found"
            stats["termination"] = status
            return PlanResult(status, reason="No complete pickup plan satisfies the motion deadband",
                              remaining=len(root.targets), optimization=stats,
                              elapsed=shared_elapsed + time.monotonic() - started,
                              trials=initial.trials + stats["trials"], expanded=initial.expanded + stats["expanded"])
        if time.monotonic() >= deadline:
            stats["termination"] = "budget_exhausted_kept_incumbent"
        try:
            checked = replay(root, best.goals, dt=.013, pickup_stops=best.pickup_stops, minimum_motion=True)
        except ValueError as error:
            return PlanResult("replay_failed", reason=str(error), remaining=len(root.targets))
        if checked.failure or checked.targets:
            return PlanResult("replay_failed", reason="Final pickup-stop replay failed", remaining=len(checked.targets))
        best.final = checked.snapshot()
        best.remaining = best.best_remaining = 0
        best.length = path_length(root.pose, best.goals)
        best.elapsed = shared_elapsed + time.monotonic() - started
        best.trials = initial.trials + stats["trials"]
        best.expanded = initial.expanded + stats["expanded"]
        best.ompl_calls = initial.ompl_calls + stats["ompl_calls"]
        best.ompl_connections = initial.ompl_connections + stats["ompl_connections"]
        best.optimization = stats
        stats["stop_count"] = len(best.pickup_stops)
        stats["saved_stops"] = baseline_count - len(best.pickup_stops) if baseline_count is not None else None
        return best

    def _plan_after_escape(self, scene, initial, started, shared_elapsed, deadline):
        from .escape import _escape_approaches

        points = list(scene.targets.values())
        intrusions = {i for i, point in enumerate(points)
                      if scene.safety_body.intersects_circle(*scene.local_point(*point), scene.target_radius)}
        trials = initial.trials if initial is not None else 0
        expanded = initial.expanded if initial is not None else 0
        found_escape = False
        for prefix in _escape_approaches(scene, points, intrusions, deadline=deadline,
                                        minimum_translation=MIN_TRANSLATION):
            try:
                escaped = replay(scene, prefix, dt=.013, minimum_motion=True)
            except ValueError:
                continue
            if escaped.failure or escaped.recovering:
                continue
            found_escape = True
            time_left = deadline - time.monotonic()
            if time_left <= 0 or trials >= self.options.max_trials or expanded >= self.options.max_expansions:
                break
            options = replace(self.options, timeout=time_left, max_trials=self.options.max_trials - trials,
                              max_expansions=self.options.max_expansions - expanded)
            result = StopsPlanner(options).plan(escaped)
            trials += result.trials
            expanded += result.expanded
            if result.status != "success":
                continue
            goals = [*prefix, *result.goals]
            stops = [{**stop, "goal_index": stop["goal_index"] + len(prefix)} for stop in result.pickup_stops]
            try:
                checked = replay(scene, goals, dt=.013, pickup_stops=stops, minimum_motion=True)
            except ValueError:
                continue
            if checked.failure or checked.targets:
                continue
            result.goals, result.pickup_stops = goals, stops
            result.final = checked.snapshot()
            result.length = path_length(scene.pose, goals)
            result.elapsed = shared_elapsed + time.monotonic() - started
            result.trials, result.expanded = trials, expanded
            result.optimization["escape_goal_count"] = len(prefix)
            return result

        exhausted = (time.monotonic() >= deadline or trials >= self.options.max_trials
                     or expanded >= self.options.max_expansions)
        reason = ("No complete pickup plan found after escaping" if found_escape
                  else "No safe escape from the initial safety intrusion")
        return PlanResult("budget_exhausted" if exhausted else "not_found", reason=reason,
                          remaining=len(points), best_remaining=len(points), trials=trials, expanded=expanded,
                          elapsed=shared_elapsed + time.monotonic() - started)
