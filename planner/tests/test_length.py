from dataclasses import replace
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from route_plan_sim import Scene
from route_plan_planner import LengthOptions, LengthPlanner, PlanResult, PlannerOptions, StaticPlanner, replay
from route_plan_planner.motion import path_length
from route_plan_planner.records import describe_scene, read_json


class LengthPlannerTest(unittest.TestCase):
    def test_front_path_gets_strictly_shorter_and_stays_feasible(self):
        scene = Scene()
        before = describe_scene(scene)
        options = PlannerOptions(timeout=2, ompl=False)
        a = StaticPlanner(options).plan(scene)
        b = LengthPlanner(options).plan(scene, initial=a)
        self.assertEqual(a.status, "success")
        self.assertEqual(b.status, "success")
        self.assertLess(b.length, a.length - .1)
        final = replay(scene, b.goals, dt=.017)
        self.assertFalse(final.failure)
        self.assertFalse(final.targets)
        self.assertAlmostEqual(b.length, path_length(scene.pose, b.goals))
        self.assertEqual(b.optimization["objective"], "translation_length_m")
        self.assertFalse(b.optimization["global_optimal"])
        self.assertEqual(describe_scene(scene), before)
        self.assertIsNone(a.optimization)

    def test_random_route_improvement_includes_all_targets(self):
        scene = Scene(scenario="random", seed=42)
        options = PlannerOptions(timeout=3, ompl=False)
        a = StaticPlanner(options).plan(scene)
        b = LengthPlanner(options).plan(scene, initial=a)
        self.assertEqual(b.status, "success")
        self.assertLess(b.length, a.length * .99)
        for dt in (.04, .7):
            final = replay(scene, b.goals, dt)
            self.assertFalse(final.failure)
            self.assertFalse(final.targets)
            self.assertEqual(set(final.collected), set(scene.targets))

    def test_exhausted_optimization_keeps_verified_incumbent(self):
        scene = Scene()
        options = PlannerOptions(timeout=.1, ompl=False)
        a = StaticPlanner(PlannerOptions(ompl=False)).plan(scene)
        a = replace(a, elapsed=1.)
        b = LengthPlanner(options).plan(scene, initial=a)
        self.assertEqual(b.status, "success")
        self.assertEqual(b.goals, a.goals)
        self.assertEqual(b.length, a.length)
        self.assertEqual(b.optimization["termination"], "budget_exhausted")
        self.assertEqual(b.optimization["improvements"], 0)

    def test_trials_budget_is_shared_with_feasibility_seed(self):
        scene = Scene()
        a = StaticPlanner(PlannerOptions(ompl=False)).plan(scene)
        b = LengthPlanner(PlannerOptions(max_trials=a.trials, ompl=False)).plan(scene, initial=a)
        self.assertEqual(b.status, "success")
        self.assertEqual(b.trials, a.trials)
        self.assertEqual(b.goals, a.goals)

    def test_final_validation_error_falls_back_to_safe_seed(self):
        scene = Scene()
        options = PlannerOptions(timeout=1, ompl=False)
        a = StaticPlanner(options).plan(scene)
        calls = 0

        def checked_replay(scene, goals):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise ValueError("Independent frame validation rejected improvement")
            return replay(scene, goals)

        with patch("route_plan_planner.length.replay", checked_replay):
            b = LengthPlanner(options).plan(scene, initial=a)
        self.assertEqual(b.status, "success")
        self.assertEqual(b.goals, a.goals)
        self.assertEqual(b.length, a.length)
        self.assertEqual(b.optimization["termination"], "improvement_replay_failed_kept_baseline")

    def test_zero_distance_rotation_has_zero_length_cost(self):
        scene = Scene()
        scene.targets.clear()
        scene.add_target(3.5, 4.7)
        seed = PlanResult("success", goals=[(3, 4.5, .5)])
        b = LengthPlanner(PlannerOptions(timeout=.2, ompl=False)).plan(scene, initial=seed)
        self.assertEqual(b.status, "success")
        self.assertEqual(b.length, 0)
        self.assertFalse(b.final["remaining"])

    def test_empty_and_failed_scenes(self):
        scene = Scene()
        scene.targets.clear()
        b = LengthPlanner(PlannerOptions(ompl=False)).plan(scene)
        self.assertEqual(b.status, "success")
        self.assertEqual(b.length, 0)
        scene.forward(100)
        self.assertEqual(LengthPlanner().plan(scene).status, "invalid_input")

    def test_invalid_initial_plan_is_rejected(self):
        with self.assertRaises(ValueError):
            LengthPlanner().plan(Scene(), initial=PlanResult("success", goals=[]))
        with self.assertRaises(ValueError):
            LengthOptions(beam_width=0)

    def test_cli_method_and_paired_comparison_records(self):
        with TemporaryDirectory() as directory:
            plan = Path(directory) / "length.json"
            process = subprocess.run([sys.executable, "-m", "route_plan_planner.cli", "solve",
                                      "--method", "length", "--timeout", "1", "--no-ompl",
                                      "--output", str(plan)], capture_output=True, text=True)
            self.assertEqual(process.returncode, 0, process.stderr)
            record = read_json(plan)
            self.assertEqual(record["planner"], "length-beam-v1")
            self.assertEqual(record["result"]["status"], "success")
            compare = Path(directory) / "compare"
            process = subprocess.run([sys.executable, "-m", "route_plan_planner.cli", "compare",
                                      "--seeds", "0:1", "--timeout", ".4", "--no-ompl",
                                      "--output", str(compare)], capture_output=True, text=True)
            self.assertEqual(process.returncode, 0, process.stderr)
            summary = read_json(compare / "summary.json")
            self.assertEqual(summary["total"], 3)
            self.assertEqual(summary["a_success"], 3)
            self.assertEqual(summary["b_success"], 3)
            self.assertEqual(summary["worse"], 0)
            for case in summary["cases"]:
                a = read_json(compare / f"A-{case['case']}.json")
                b = read_json(compare / f"B-{case['case']}.json")
                self.assertEqual(a["initial"], b["initial"])
                self.assertLessEqual(b["result"]["length"], a["result"]["length"] + 1e-8)


if __name__ == "__main__":
    unittest.main()
