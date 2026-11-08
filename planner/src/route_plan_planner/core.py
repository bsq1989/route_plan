"""Feasibility-first DFS with retained alternatives and simulator transitions."""

from dataclasses import asdict, dataclass, field
import math
import time
from typing import Iterator, Protocol

from route_plan_sim import Scene
from route_plan_sim.scene import EPS, normalize

from .motion import (DirectMotion, MotionBackend, Pose, Transition, compact_goals,
                     path_length, replay)


@dataclass(frozen=True)
class PlannerOptions:
    timeout: float = 30.
    max_expansions: int = 2000
    max_trials: int = 30000
    headings: int = 24
    relocation_steps: tuple[float, ...] = (.4, .8, 1.4)
    max_relocations: int = 3
    ompl: bool = True
    ompl_seconds: float = .4
    ompl_attempts: int = 4
    seed: int = 42

    def __post_init__(self):
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        if self.max_expansions < 1 or self.max_trials < 1 or self.headings < 4:
            raise ValueError("search budgets must be positive; headings must be >=4")
        if self.max_relocations < 0 or self.ompl_attempts < 0:
            raise ValueError("attempt counts must be nonnegative")
        if not math.isfinite(self.ompl_seconds) or self.ompl_seconds <= 0:
            raise ValueError("ompl_seconds must be positive and finite")
        if any(not math.isfinite(x) or x <= 0 for x in self.relocation_steps):
            raise ValueError("relocation steps must be positive and finite")


@dataclass
class PlanResult:
    status: str
    goals: list[Pose] = field(default_factory=list)
    elapsed: float = 0.
    expanded: int = 0
    trials: int = 0
    ompl_calls: int = 0
    ompl_connections: int = 0
    backtracks: int = 0
    remaining: int = 0
    best_remaining: int = 0
    length: float = 0.
    reason: str | None = None
    final: dict | None = None
    diagnostic: dict | None = None
    optimization: dict | None = None

    def to_dict(self) -> dict:
        return asdict(self)


class TaskPlanner(Protocol):
    def plan(self, scene: Scene) -> PlanResult: ...


class _BudgetExceeded(Exception):
    pass


class StaticPlanner:
    """A baseline, not a completeness or global optimality guarantee.

    The task and motion backend interfaces permit replacing either independently.
    No branch is executed on the caller's scene. Quantized visited keys and
    bounded candidate generation deliberately trade completeness for demo speed.
    """

    def __init__(self, options: PlannerOptions | None = None,
                 motion: MotionBackend | None = None,
                 fallback: MotionBackend | None = None):
        self.options = options or PlannerOptions()
        self.motion = motion or DirectMotion()
        if fallback is None and self.options.ompl:
            from .ompl_motion import OmplMotion
            fallback = OmplMotion(self.options.ompl_seconds, self.options.seed)
        self.fallback = fallback

    def _check_budget(self):
        if (time.monotonic() >= self.deadline or self.result.trials >= self.options.max_trials
                or self.result.expanded >= self.options.max_expansions):
            raise _BudgetExceeded

    def _candidates(self, scene: Scene) -> list[Pose]:
        candidates = []
        pickup_reach = max(math.hypot(x, y) for x, y in scene.collector.corners) + scene.target_radius
        # Disk centres may be outside the tool but must clear the safety body.
        near = max(scene.collector.xmin - scene.target_radius,
                   scene.safety_body.xmax + scene.target_radius) + .015
        far = scene.collector.xmax + scene.target_radius - min(.015, (scene.collector.xmax - scene.collector.xmin) / 10)
        depths = (far, (near + far) / 2) if near < far else ((scene.collector.xmin + scene.collector.xmax) / 2,)
        offset = (scene.collector.ymin + scene.collector.ymax) / 2
        for tx, ty in scene.targets.values():
            bearing = math.atan2(ty - scene.pose[1], tx - scene.pose[0])
            angles = [bearing, scene.pose[2]] + [i * math.tau / self.options.headings
                                               for i in range(self.options.headings)]
            dx, dy = tx - scene.pose[0], ty - scene.pose[1]
            if math.hypot(dx, dy) <= pickup_reach + EPS:
                # Circle contact can occur at a rounded collector corner. Add
                # corner-aligned yaws as well as the ordinary heading samples.
                pickup_angles = angles + [bearing - math.atan2(cy, cx)
                                          for cx, cy in scene.collector.corners]
                for angle in pickup_angles:
                    c, s = math.cos(angle), math.sin(angle)
                    if scene.collector.intersects_circle(c * dx + s * dy, -s * dx + c * dy,
                                                         scene.target_radius):
                        candidates.append((*scene.pose[:2], normalize(angle)))
            for angle in angles:
                c, s = math.cos(angle), math.sin(angle)
                for depth in depths:
                    candidates.append((tx - c * depth + s * offset,
                                       ty - s * depth - c * offset, normalize(angle)))
        # A cheap ordering, not an optimization objective or a pruning proof.
        return sorted(set(candidates), key=lambda g: math.dist(scene.pose[:2], g[:2])
                      + .12 * abs(normalize(g[2] - scene.pose[2])))

    def _successors(self, scene: Scene, relocations: int) -> Iterator[Transition]:
        candidates = self._candidates(scene)
        for goal in candidates:
            self._check_budget()
            self.result.trials += 1
            transition = self.motion.connect(scene, goal, self.deadline)
            if transition and len(transition.scene.targets) < len(scene.targets):
                yield transition
        if self.fallback:
            attempts = 0
            for goal in candidates:
                self._check_budget()
                if attempts >= self.options.ompl_attempts:
                    break
                # Do not spend a scarce sampling budget on an invalid endpoint.
                endpoint = scene.clone()
                endpoint.pose = goal
                endpoint.path = [goal[:2]]
                endpoint.advance()
                if endpoint.failure:
                    continue
                attempts += 1
                self.result.trials += 1
                self.result.ompl_calls += 1
                transition = self.fallback.connect(scene, goal, self.deadline)
                if transition and len(transition.scene.targets) < len(scene.targets):
                    self.result.ompl_connections += 1
                    yield transition
        if relocations < self.options.max_relocations:
            x, y, yaw = scene.pose
            for distance in self.options.relocation_steps:
                for index in range(16):
                    self._check_budget()
                    angle = yaw + index * math.tau / 16
                    self.result.trials += 1
                    transition = self.motion.connect(
                        scene, (x + distance * math.cos(angle), y + distance * math.sin(angle),
                                normalize(angle)), self.deadline)
                    if transition:
                        yield transition

    @staticmethod
    def _key(scene: Scene, relocations: int) -> tuple:
        return (round(scene.pose[0], 3), round(scene.pose[1], 3), round(scene.pose[2], 3),
                tuple(sorted(scene.targets)), relocations)

    def plan(self, scene: Scene) -> PlanResult:
        started = time.monotonic()
        self.deadline = started + self.options.timeout
        self.result = PlanResult("not_found", remaining=len(scene.targets), best_remaining=len(scene.targets))
        if scene.failure:
            self.result.status = "invalid_input"
            self.result.reason = "Reset the failed scene before planning"
            return self.result
        # Frames retain their lazy alternatives, so failure really can backtrack.
        root = scene.clone()
        stack = [(root, [], 0, self._successors(root, 0))]
        visited = {self._key(root, 0)}
        try:
            while stack:
                current, goals, relocations, successors = stack[-1]
                if len(current.targets) < self.result.best_remaining:
                    self.result.best_remaining = len(current.targets)
                    self.result.diagnostic = {"state": current.snapshot(),
                                              "partial_goals": compact_goals(scene.pose, goals)}
                if not current.targets:
                    goals = compact_goals(scene.pose, goals)
                    try:
                        final = replay(scene, goals)
                    except ValueError as error:
                        self.result.status = "replay_failed"
                        self.result.reason = f"Illegal pose command during replay: {error}"
                        break
                    self.result.goals = goals
                    self.result.final = final.snapshot()
                    self.result.remaining = len(final.targets)
                    self.result.length = path_length(scene.pose, goals)
                    self.result.status = "success" if not final.failure and not final.targets else "replay_failed"
                    self.result.reason = (final.failure or
                                          (f"Replay left {len(final.targets)} targets" if final.targets else None))
                    self.result.diagnostic = None
                    break
                self._check_budget()
                transition = next(successors, None)
                if transition is None:
                    stack.pop()
                    self.result.backtracks += 1
                    continue
                next_relocations = (0 if len(transition.scene.targets) < len(current.targets)
                                    else relocations + 1)
                key = self._key(transition.scene, next_relocations)
                if key in visited:
                    continue
                visited.add(key)
                self.result.expanded += 1
                child = transition.scene
                stack.append((child, goals + transition.goals, next_relocations,
                              self._successors(child, next_relocations)))
        except _BudgetExceeded:
            self.result.status = "budget_exhausted"
            self.result.reason = "Search budget reached; this is not a proof of infeasibility"
        if self.result.status == "not_found":
            self.result.reason = "Bounded candidates exhausted; this is not a proof of infeasibility"
        self.result.elapsed = time.monotonic() - started
        return self.result
