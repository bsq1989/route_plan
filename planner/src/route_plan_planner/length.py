"""Anytime path-length optimization with a verified feasible incumbent.

Beam truncation and finite candidate sampling are intentional: this planner
improves feasible paths and makes no global shortest-path claim.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
import time

from route_plan_sim import Scene

from .core import PlanResult, PlannerOptions, StaticPlanner, _BudgetExceeded
from .motion import compact_goals, execute, path_length, replay


@dataclass(frozen=True)
class LengthOptions:
    beam_width: int = 4
    branching: int = 6
    trials_per_node: int = 64
    variants_per_collection: int = 2
    shortcut_trials: int = 40
    route_trials: int = 120

    def __post_init__(self):
        if min(self.beam_width, self.branching, self.trials_per_node,
               self.variants_per_collection) < 1 or min(self.shortcut_trials, self.route_trials) < 0:
            raise ValueError("Length search counts must be positive; local trial limits may be zero")


class _NodeLimit(Exception):
    pass


class _LengthSearch(StaticPlanner):
    node_limit = math.inf

    def _check_budget(self):
        super()._check_budget()
        if self.result.trials >= self.node_limit:
            raise _NodeLimit

    def _candidates(self, scene):
        # Rotation is free in the length objective. No angular cost is added.
        return sorted(super()._candidates(scene),
                      key=lambda goal: math.dist(scene.pose[:2], goal[:2]))


class LengthPlanner:
    def __init__(self, options: PlannerOptions | None = None,
                 length_options: LengthOptions | None = None, motion=None, fallback=None):
        self.options = options or PlannerOptions()
        self.length_options = length_options or LengthOptions()
        self.motion = motion
        self.fallback = fallback

    def plan(self, scene: Scene, initial: PlanResult | None = None) -> PlanResult:
        started = time.monotonic()
        shared_elapsed = initial.elapsed if initial is not None else 0.
        deadline = started + max(0., self.options.timeout - shared_elapsed)
        if initial is None:
            initial = StaticPlanner(self.options, self.motion, self.fallback).plan(scene)
        else:
            if initial.status != "success":
                return deepcopy(initial)
            checked = replay(scene, initial.goals)
            if checked.failure or checked.targets:
                raise ValueError("Initial plan is not feasible for this scene")
        best = deepcopy(initial)
        if best.status != "success":
            return best
        best.length = path_length(scene.pose, best.goals)
        baseline_length = best.length
        baseline_goals = list(best.goals)
        stats = {"method": "length-beam-v1", "objective": "translation_length_m",
                 "baseline_length": baseline_length, "improvements": 0,
                 "shortcut_trials": 0, "route_trials": 0, "route_connections": 0,
                 "expanded": 0, "trials": 0,
                 "termination": "candidates_exhausted", "global_optimal": False}
        best.optimization = stats

        def spent_trials():
            return initial.trials + stats["shortcut_trials"] + stats["route_connections"]

        def accept(goals):
            goals = compact_goals(scene.pose, goals)
            cost = path_length(scene.pose, goals)
            if cost >= best.length - 1e-8:
                return
            # Check the whole suffix too: skipping an earlier collection can
            # make an otherwise identical later motion collide.
            try:
                final = execute(scene.clone(), goals)
            except ValueError:
                return
            if final.failure or final.targets:
                return
            best.goals, best.length = goals, cost
            stats["improvements"] += 1
            return True

        # First try removing detours while preserving the later endpoint yaw.
        # Prefixes include their actual target deletions; no waypoint-only test.
        prefix = scene.clone()
        prefix_goals = []
        shortcut_deadline = min(deadline, time.monotonic() + .15)
        connector = _LengthSearch(self.options, self.motion, self.fallback).motion
        for i in range(len(baseline_goals)):
            if (time.monotonic() >= shortcut_deadline or stats["shortcut_trials"] >= self.length_options.shortcut_trials
                    or spent_trials() >= self.options.max_trials):
                break
            for j in range(len(baseline_goals) - 1, i + 1, -1):
                if (time.monotonic() >= shortcut_deadline or stats["shortcut_trials"] >= self.length_options.shortcut_trials
                        or spent_trials() >= self.options.max_trials):
                    break
                stats["shortcut_trials"] += 1
                transition = connector.connect(prefix, baseline_goals[j], shortcut_deadline)
                if transition:
                    accept(prefix_goals + transition.goals + baseline_goals[j + 1:])
            execute(prefix, [baseline_goals[i]])
            prefix_goals.append(baseline_goals[i])

        search = _LengthSearch(self.options, self.motion, self.fallback)
        radius = max(math.hypot(x, y) for x, y in scene.collector.corners)
        # Reorder actual collection events, then regenerate approach poses.
        # Existing pose/yaw choices need not remain useful after a reversal.
        route_deadline = min(deadline, time.monotonic() + .7 * max(0., deadline - time.monotonic()))
        rebuilt_orders = set()
        while (time.monotonic() < route_deadline and stats["route_trials"] < self.length_options.route_trials
               and spent_trials() < self.options.max_trials):
            order = []
            trace = scene.clone()
            for goal in best.goals:
                before = set(trace.targets)
                execute(trace, [goal])
                order.extend(sorted(before - set(trace.targets)))
            proposals = [(0., tuple(order))]
            for i in range(len(order) - 1):
                previous = scene.pose[:2] if i == 0 else scene.targets[order[i - 1]]
                for j in range(i + 1, len(order)):
                    saving = math.dist(previous, scene.targets[order[i]]) - math.dist(previous, scene.targets[order[j]])
                    if j + 1 < len(order):
                        saving += (math.dist(scene.targets[order[j]], scene.targets[order[j + 1]])
                                   - math.dist(scene.targets[order[i]], scene.targets[order[j + 1]]))
                    if saving > 1e-8:
                        proposals.append((-saving, tuple(order[:i] + list(reversed(order[i:j + 1])) + order[j + 1:])))
            improved = False
            for _, route in sorted(proposals):
                if time.monotonic() >= route_deadline or stats["route_trials"] >= self.length_options.route_trials:
                    break
                if route in rebuilt_orders:
                    continue
                rebuilt_orders.add(route)
                stats["route_trials"] += 1
                trial, goals, cost = scene.clone(), [], 0.
                for index, target in enumerate(route):
                    if target not in trial.targets:
                        continue
                    if time.monotonic() >= route_deadline or spent_trials() >= self.options.max_trials:
                        break
                    focus = trial.clone()
                    focus.targets = {target: trial.targets[target]}
                    next_target = next((key for key in route[index + 1:] if key in trial.targets), None)

                    def score(pose, distance):
                        ahead = (max(0., math.dist(pose[:2], trial.targets[next_target]) - radius)
                                 if next_target is not None else 0.)
                        return distance + ahead

                    candidates = sorted(search._candidates(focus),
                                        key=lambda goal: score(goal, math.dist(trial.pose[:2], goal[:2])))
                    choices = []
                    for goal in candidates:
                        if time.monotonic() >= route_deadline or spent_trials() >= self.options.max_trials:
                            break
                        stats["route_connections"] += 1
                        transition = connector.connect(trial, goal, route_deadline)
                        if transition and target not in transition.scene.targets:
                            distance = path_length(trial.pose, transition.goals)
                            choices.append((score(transition.scene.pose, distance), distance, transition))
                            if len(choices) >= 3:
                                break
                    if not choices:
                        break
                    _, distance, transition = min(choices, key=lambda item: item[0])
                    cost += distance
                    if cost >= best.length - 1e-8:
                        break
                    trial = transition.scene
                    goals.extend(transition.goals)
                    if not trial.targets:
                        if accept(goals):
                            improved = True
                        break
                if improved:
                    break
            if not improved:
                break

        search.deadline = deadline
        search.result = PlanResult("not_found")
        search.result.trials = spent_trials()
        search.result.expanded = initial.expanded
        trials_before_beam = search.result.trials
        # The collector's enclosing circle relaxes orientation and collision
        # constraints. Its region-MST is a translation-distance lower bound.
        pair_gap = {(i, j): max(0., math.dist(a, b) - 2 * radius)
                    for i, a in scene.targets.items() for j, b in scene.targets.items()}

        def bound(current):
            remaining = set(current.targets)
            if not remaining:
                return 0.
            distances = {key: max(0., math.dist(current.pose[:2], current.targets[key]) - radius)
                         for key in remaining}
            farthest = max(distances.values())
            total = 0.
            while remaining:
                key = min(remaining, key=distances.__getitem__)
                total += distances[key]
                remaining.remove(key)
                for other in remaining:
                    distances[other] = min(distances[other], pair_gap[key, other])
            return max(total, farthest)

        # A layer represents collection/motion decisions, not one target each:
        # a single transition may collect many, or temporarily collect none.
        layer = [(scene.clone(), [], 0., 0)]
        seen = {search._key(scene, 0): 0.}
        try:
            while layer:
                next_layer = []
                for current, goals, cost, relocations in layer:
                    search._check_budget()
                    if cost + bound(current) >= best.length - 1e-8:
                        continue
                    search.result.expanded += 1
                    search.node_limit = search.result.trials + self.length_options.trials_per_node
                    groups = {}
                    accepted = 0
                    try:
                        for transition in search._successors(current, relocations):
                            child = transition.scene
                            delta = path_length(current.pose, transition.goals)
                            new_cost = cost + delta
                            if new_cost >= best.length - 1e-8:
                                continue
                            new_goals = goals + transition.goals
                            if not child.targets:
                                accept(new_goals)
                                continue
                            if new_cost + bound(child) >= best.length - 1e-8:
                                continue
                            collection = tuple(sorted(set(current.targets) - set(child.targets)))
                            if groups.get(collection, 0) >= self.length_options.variants_per_collection:
                                continue
                            next_relocations = 0 if collection else relocations + 1
                            key = search._key(child, next_relocations)
                            if new_cost >= seen.get(key, math.inf) - 1e-8:
                                continue
                            seen[key] = new_cost
                            groups[collection] = groups.get(collection, 0) + 1
                            next_layer.append((child, new_goals, new_cost, next_relocations))
                            accepted += 1
                            if accepted >= self.length_options.branching:
                                break
                    except _NodeLimit:
                        pass
                    finally:
                        search.node_limit = math.inf
                next_layer.sort(key=lambda item: item[2] + bound(item[0]))
                layer = next_layer[:self.length_options.beam_width]
        except _BudgetExceeded:
            stats["termination"] = "budget_exhausted"
        stats["expanded"] = search.result.expanded - initial.expanded
        stats["trials"] = search.result.trials - trials_before_beam
        stats["ompl_calls"] = search.result.ompl_calls
        # Final independent frame replay is mandatory even after fast trial
        # validation. A failed improvement must never replace the valid seed.
        try:
            final = replay(scene, best.goals)
        except ValueError as error:
            final = None
            stats["validation_failure"] = str(error)
        if final is None or final.failure or final.targets:
            best = deepcopy(initial)
            best.optimization = stats
            stats["termination"] = "improvement_replay_failed_kept_baseline"
            final = replay(scene, best.goals)
        best.final = final.snapshot()
        best.remaining = len(final.targets)
        best.best_remaining = len(final.targets)
        best.length = path_length(scene.pose, best.goals)
        stats["saved_length"] = baseline_length - best.length
        stats["saved_fraction"] = (1 - best.length / baseline_length) if baseline_length > 0 else 0.
        best.trials = search.result.trials
        best.expanded = search.result.expanded
        best.ompl_calls = initial.ompl_calls + search.result.ompl_calls
        best.ompl_connections = initial.ompl_connections + search.result.ompl_connections
        best.elapsed = shared_elapsed + time.monotonic() - started
        return best
