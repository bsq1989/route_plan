from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import unittest

from route_plan_sim import Scene
from route_plan_planner import PlannerOptions, StaticPlanner, replay
from route_plan_planner.motion import Transition, steer
from route_plan_planner.ompl_motion import OmplMotion
from route_plan_planner.records import describe_scene, read_json, restore_scene, write_json
from route_plan_planner.stops import at_stop_scene

FIXTURES = Path(__file__).parent / "fixtures" / "static_cases.json"
RECTANGULAR_ROBOT = Path(__file__).resolve().parents[2] / "sim/tests/fixtures/robot-rectangular.yaml"


class StaticBaselineTest(unittest.TestCase):
    def empty(self, robot_config=None):
        scene = Scene(robot_config=robot_config)
        scene.targets.clear()
        return scene

    def test_frozen_cases_and_independent_witnesses(self):
        for case in read_json(FIXTURES)["cases"]:
            with self.subTest(case=case["name"]):
                scene = restore_scene(case["initial"])
                before = describe_scene(scene)
                # A frozen feasible trajectory prevents accepting an unsolvable
                # layout into the mandatory regression set by mistake.
                witness = replay(scene, case["witness"])
                self.assertFalse(witness.failure)
                self.assertFalse(witness.targets)
                result = StaticPlanner(PlannerOptions(**case["options"])).plan(scene)
                self.assertEqual(result.status, "success", result.reason)
                self.assertFalse(result.final["remaining"])
                self.assertIsNone(result.final["failure"])
                self.assertEqual(describe_scene(scene), before)
                if case["name"] == "backtrack_corner_last":
                    self.assertGreater(result.backtracks, 0)

    def test_all_presets_and_frame_size_independence(self):
        for scenario in ("front", "surround", "random"):
            scene = Scene(scenario=scenario, seed=42)
            result = StaticPlanner().plan(scene)
            self.assertEqual(result.status, "success")
            for dt in (.013, .17, 2.):
                with self.subTest(scenario=scenario, dt=dt):
                    final = replay(scene, result.goals, dt)
                    self.assertFalse(final.failure)
                    self.assertFalse(final.targets)
                    for actual, expected in zip(final.pose, result.final["pose"]):
                        self.assertAlmostEqual(actual, expected, places=7)

    def test_budget_exhaustion_does_not_execute_partial_plan(self):
        scene = Scene()
        before = describe_scene(scene)
        result = StaticPlanner(PlannerOptions(max_trials=1, ompl=False)).plan(scene)
        self.assertEqual(result.status, "budget_exhausted")
        self.assertEqual(result.goals, [])
        self.assertEqual(result.remaining, len(scene.targets))
        self.assertEqual(describe_scene(scene), before)

    def test_failed_input_is_reported_without_reset(self):
        scene = self.empty()
        scene.forward(100)
        before = scene.snapshot()
        result = StaticPlanner().plan(scene)
        self.assertEqual(result.status, "invalid_input")
        self.assertEqual(scene.snapshot(), before)

    def test_replacement_backend_cannot_bypass_controller_rules(self):
        class InvalidBackend:
            def connect(self, scene, goal, deadline):
                # Deliberately claim collection while commanding a rear goal.
                trial = scene.clone()
                trial.targets.clear()
                return Transition(trial, [(2, 4.5, 0)])

        scene = self.empty()
        scene.add_target(5, 4.5)
        result = StaticPlanner(PlannerOptions(ompl=False), motion=InvalidBackend()).plan(scene)
        self.assertEqual(result.status, "replay_failed")
        self.assertIn("behind", result.reason)
        self.assertEqual(len(scene.targets), 1)

    def test_swept_rotation_collision_is_not_accepted(self):
        scene = at_stop_scene(self.empty())
        scene.add_target(3, 4.99)
        self.assertIsNone(steer(scene, (3, 4.5, math.pi)))
        self.assertIsNone(scene.failure)

    def test_rotation_can_collect_without_translation(self):
        scene = Scene(robot_config=Path(__file__).resolve().parents[2] / "sim/tests/fixtures/robot-v0.2.4.yaml")
        scene.targets.clear()
        scene.add_target(3.45, 4.8)
        self.assertEqual(len(scene.targets), 1)
        final = replay(scene, [(3, 4.5, .5)])
        self.assertFalse(final.failure)
        self.assertFalse(final.targets)
        self.assertEqual(final.pose[:2], scene.pose[:2])

    def test_multiple_points_collected_by_one_translation(self):
        scene = self.empty()
        for x in (4, 4.2, 4.4):
            scene.add_target(x, 4.5)
        final = replay(scene, [(4.2, 4.5, 0)])
        self.assertFalse(final.targets)
        self.assertFalse(final.failure)

    def test_record_roundtrip_uses_stored_vehicle_and_preserves_ids(self):
        scene = self.empty(RECTANGULAR_ROBOT)
        scene.set_dimensions(**(scene.dimensions | {"collector_width": .31}))
        scene.add_target(5, 4.5)
        scene.add_target(6, 6)
        first_id = next(iter(scene.targets))
        scene.targets[21] = scene.targets.pop(first_id)
        expected_ids = set(scene.targets)
        record = json.loads(json.dumps(describe_scene(scene)))
        restored = restore_scene(record)
        self.assertEqual(restored.snapshot(), scene.snapshot())
        self.assertEqual(describe_scene(restored)["vehicle_config"]["body"]["height"], .169)
        result = StaticPlanner().plan(restored)
        self.assertEqual(result.status, "success")
        final = replay(restored, result.goals)
        self.assertEqual(set(final.collected), expected_ids)
        self.assertEqual(restored.dimensions["collector_width"], .31)

    def test_records_preserve_circle_size_and_legacy_points_explicitly(self):
        scene = Scene(target_diameter=.16)
        scene.targets.clear()
        scene.add_target(5, 4.5)
        record = json.loads(json.dumps(describe_scene(scene)))
        restored = restore_scene(record)
        self.assertEqual(restored.target_diameter, .16)
        self.assertEqual(restored.snapshot(), scene.snapshot())
        # A conflicting stored geometry must never silently replay as another size.
        record["state"]["target_diameter"] = .1
        with self.assertRaisesRegex(ValueError, "diameter"):
            restore_scene(record)
        # Historical point plans lack both fields; preserve their original model.
        record["state"].pop("target_diameter")
        record["world_config"]["custom"].pop("target_diameter")
        restored = restore_scene(record)
        self.assertEqual(restored.target_diameter, 0)

    def test_circle_plan_replays_with_geometry_and_ids_preserved(self):
        scene = self.empty()
        for point in ((4, 4.5), (5, 5)):
            scene.add_target(*point)
        before = describe_scene(scene)
        result = StaticPlanner(PlannerOptions(ompl=False)).plan(scene)
        self.assertEqual(result.status, "success", result.reason)
        restored = restore_scene(json.loads(json.dumps(before)))
        for dt in (.013, .7, 10):
            final = replay(restored, result.goals, dt=dt)
            self.assertIsNone(final.failure)
            self.assertFalse(final.targets)
            self.assertEqual(set(final.collected), set(scene.targets))
            self.assertEqual(final.target_diameter, .1)

    def test_ompl_bypasses_blocker_and_final_approach_collects(self):
        scene = self.empty(RECTANGULAR_ROBOT)
        scene.add_target(4.5, 4.8)
        scene.add_target(6.5, 4.5)
        blocker_id, target_id = scene.targets
        # Derive the approach from the active collector and physical radius.
        goal = (6.5 - scene.collector.xmax - scene.target_radius + .015, 4.5, 0)
        self.assertIsNone(steer(scene, goal))
        connected = OmplMotion(seconds=4, seed=42).connect(scene, goal, time.monotonic() + 5)
        self.assertIsNotNone(connected)
        self.assertEqual(connected.source, "ompl")
        self.assertNotIn(target_id, connected.scene.targets)
        self.assertIn(blocker_id, connected.scene.targets)
        rest = StaticPlanner().plan(connected.scene)
        self.assertEqual(rest.status, "success")
        final = replay(scene, connected.goals + rest.goals)
        self.assertFalse(final.failure)
        self.assertFalse(final.targets)

    def test_cli_saved_plan_replays_without_replanning(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "plan.json"
            solve = subprocess.run([sys.executable, "-m", "route_plan_planner.cli", "solve",
                                    "--output", str(path)], capture_output=True, text=True)
            self.assertEqual(solve.returncode, 0, solve.stderr)
            saved = read_json(path)
            self.assertEqual(saved["result"]["status"], "success")
            play = subprocess.run([sys.executable, "-m", "route_plan_planner.cli", "replay", str(path)],
                                  capture_output=True, text=True)
            self.assertEqual(play.returncode, 0, play.stderr)
            self.assertIn("replay success", play.stdout)
            saved["result"]["goals"] = []
            write_json(path, saved)
            bad = subprocess.run([sys.executable, "-m", "route_plan_planner.cli", "replay", str(path)],
                                 capture_output=True, text=True)
            self.assertNotEqual(bad.returncode, 0)

    def test_invalid_options_and_replay_step(self):
        for changes in ({"timeout": float("nan")}, {"max_trials": 0}, {"headings": 2},
                        {"relocation_steps": (-1,)}, {"ompl_seconds": 0}):
            with self.assertRaises(ValueError):
                replace(PlannerOptions(), **changes)
        with self.assertRaises(ValueError):
            replay(self.empty(), [(4, 4.5, 0)], dt=0)


if __name__ == "__main__":
    unittest.main()
