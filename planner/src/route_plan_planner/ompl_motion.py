"""OMPL RRT with simulator-checked, turn/forward/turn directed edges.

The local tree conservatively preserves the current target set. Collection is
allowed in the final approach and is propagated back to the task search. This
keeps tree states Markovian without pretending a fixed SE2 tree models deletions.
"""

import time

from route_plan_sim import Scene

from .motion import Pose, Transition, staging_pose, steer

_rng_initialized = False


class OmplMotion:
    def __init__(self, seconds: float = .4, seed: int = 42, *, minimum_motion: bool = False):
        self.seconds = seconds
        self.seed = seed
        self.minimum_motion = minimum_motion

    def connect_pose(self, scene: Scene, goal: Pose, deadline: float) -> Transition | None:
        from ompl import base as ob, geometric as og, util as ou

        seconds = min(self.seconds, deadline - time.monotonic())
        if seconds <= 0:
            return None
        ou.setLogLevel(ou.LOG_ERROR)
        # OMPL seeds its global RNG only before the first RNG is created.
        # Full exported goals permit replay regardless of time-budget variability.
        global _rng_initialized
        if not _rng_initialized:
            ou.RNG.setSeed(self.seed)
            _rng_initialized = True
        space = ob.SE2StateSpace()
        bounds = ob.RealVectorBounds(2)
        xmin, ymin, xmax, ymax = scene.map_bounds
        bounds.setLow(0, xmin + .001)
        bounds.setLow(1, ymin + .001)
        bounds.setHigh(0, xmax - .001)
        bounds.setHigh(1, ymax - .001)
        space.setBounds(bounds)

        def pose(state):
            return state.getX(), state.getY(), state.getYaw()

        def at(state):
            trial = scene.clone()
            trial.pose = pose(state)
            trial.path = [trial.pose[:2]]
            trial.advance()
            return trial

        def valid(state):
            if not scene.contains_motion_point(state.getX(), state.getY()):
                return False
            trial = at(state)
            return not trial.failure and trial.targets == scene.targets

        setup = og.SimpleSetup(space)
        setup.setStateValidityChecker(valid)

        minimum_motion = self.minimum_motion

        class Validator(ob.MotionValidator):
            def checkMotion(self, start, end):
                transition = steer(at(start), pose(end), minimum_motion=minimum_motion)
                return bool(transition and transition.scene.targets == scene.targets)

        validator = Validator(setup.getSpaceInformation())
        setup.getSpaceInformation().setMotionValidator(validator)
        start = space.allocState()
        end = space.allocState()
        for state, values in ((start, scene.pose), (end, goal)):
            state.setX(values[0])
            state.setY(values[1])
            state.setYaw(values[2])
        if not valid(start) or not valid(end):
            return None
        setup.setStartAndGoalStates(start, end, 1e-6)
        planner = og.RRT(setup.getSpaceInformation())
        planner.setRange(1.2)
        planner.setGoalBias(.2)
        setup.setPlanner(planner)
        setup.solve(max(.001, min(seconds, deadline - time.monotonic())))
        if not setup.haveExactSolutionPath():
            return None
        trial = scene.clone()
        goals = []
        for state in setup.getSolutionPath().getStates()[1:]:
            transition = steer(trial, pose(state), minimum_motion=minimum_motion)
            if transition is None or transition.scene.targets != scene.targets:
                return None
            trial = transition.scene
            goals.extend(transition.goals)
        return Transition(trial, goals, "ompl")

    def connect(self, scene: Scene, goal: Pose, deadline: float) -> Transition | None:
        if not scene.contains_motion_point(*goal[:2]):
            return None
        # Approach from a staging pose outside the collector's contact region.
        staging = staging_pose(scene, goal, minimum_motion=self.minimum_motion)
        reposition = self.connect_pose(scene, staging, deadline)
        if reposition is not None:
            approach = steer(reposition.scene, goal, minimum_motion=self.minimum_motion)
            if approach is not None:
                return Transition(approach.scene, reposition.goals + approach.goals, "ompl")
        # A fence can exclude the backward staging pose even when the stop is
        # reachable via a different heading and a final in-place turn.
        if (scene.fence is not None or scene.motion_circle is not None) and scene.collection_mode == "at_stop":
            return self.connect_pose(scene, goal, deadline)
        return None
