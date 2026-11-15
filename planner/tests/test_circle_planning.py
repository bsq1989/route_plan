import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

from route_plan_sim import Scene
from route_plan_planner import PlannerOptions, StaticPlanner, replay
from route_plan_planner.records import describe_scene, read_json, restore_scene

PREVIOUS_ROBOT = Path(__file__).resolve().parents[2] / "sim/tests/fixtures/robot-v0.2.4.yaml"


class CirclePlanningTest(unittest.TestCase):
    def empty(self, diameter=.1):
        scene = Scene(target_diameter=diameter, robot_config=PREVIOUS_ROBOT)
        scene.targets.clear()
        return scene

    def test_overlap_group_roundtrip_and_replay_preserve_every_id(self):
        scene = self.empty()
        ids = scene.add_targets([(5, 4.5), (5.08, 4.5), (5.16, 4.5), (7, 5)])
        record = json.loads(json.dumps(describe_scene(scene)))
        restored = restore_scene(record)
        self.assertEqual(restored.targets, scene.targets)
        self.assertEqual(restored.target_groups, scene.target_groups)
        result = StaticPlanner().plan(restored)
        self.assertEqual(result.status, "success", result.reason)
        for dt in (.013, .7):
            final = replay(restored, result.goals, dt=dt)
            self.assertIsNone(final.failure)
            self.assertFalse(final.targets)
            self.assertEqual(set(final.collected), set(ids))

    def test_plan_uses_circle_edge_without_driving_centre_into_collector(self):
        scene = self.empty()
        scene.add_target(5, 4.5)
        result = StaticPlanner(PlannerOptions(ompl=False)).plan(scene)
        self.assertEqual(result.status, "success")
        final = replay(scene, result.goals)
        self.assertFalse(final.targets)
        self.assertIsNone(final.failure)
        # At the service pose the disk touches the tool but its centre remains
        # beyond the physical tool front. A point-only adapter would miss it.
        local = final.local_point(5, 4.5)
        self.assertGreater(local[0], scene.collector.xmax)
        self.assertTrue(scene.collector.intersects_circle(*local, scene.target_radius))
        self.assertFalse(scene.collector.contains(*local))
        self.assertEqual(scene.body_margin, .1)

    def test_in_place_pickup_including_rounded_corner_and_custom_radius(self):
        for diameter, distance in ((.1, .53), (.16, .59)):
            with self.subTest(diameter=diameter):
                scene = self.empty(diameter)
                angle = math.radians(40)
                scene.add_target(3 + distance * math.cos(angle), 4.5 + distance * math.sin(angle))
                self.assertEqual(len(scene.targets), 1)
                before = scene.snapshot()
                result = StaticPlanner(PlannerOptions(timeout=2, ompl=False)).plan(scene)
                self.assertEqual(result.status, "success")
                self.assertAlmostEqual(result.length, 0.)
                self.assertTrue(all(goal[:2] == scene.pose[:2] for goal in result.goals))
                restored = restore_scene(json.loads(json.dumps(describe_scene(scene))))
                for dt in (.013, .7):
                    final = replay(restored, result.goals, dt=dt)
                    self.assertFalse(final.targets)
                    self.assertIsNone(final.failure)
                    self.assertEqual(final.target_diameter, diameter)
                self.assertEqual(scene.snapshot(), before)

    def test_corner_of_expanded_box_is_not_a_circle_contact(self):
        scene = self.empty()
        point = (scene.collector.xmax + .04, scene.collector.ymax + .04)
        self.assertFalse(scene.collector.intersects_circle(*point, scene.target_radius))
        scene.add_target(scene.pose[0] + point[0], scene.pose[1] + point[1])
        candidates = StaticPlanner(PlannerOptions(ompl=False))._candidates(scene)
        self.assertNotIn(scene.pose, candidates)

    def test_cli_selected_circle_cases_preserve_geometry(self):
        with TemporaryDirectory() as directory:
            output = Path(directory) / "regression"
            # Exercise the same selection path as the ten-case regression with
            # two real solves; keep unit tests independent of the full batch.
            command = [sys.executable, "-m", "route_plan_planner.cli", "compare",
                       "--seed-list", "42,7", "--random-only", "--timeout", "1", "--no-ompl",
                       "--output", str(output)]
            completed = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            summary = read_json(output / "summary.json")
            self.assertEqual(summary["total"], 2)
            self.assertEqual([case["seed"] for case in summary["cases"]], [42, 7])
            self.assertEqual(summary["geometry"]["target_shape"], "circle")
            self.assertEqual(summary["geometry"]["target_diameter"], .1)
            self.assertAlmostEqual(summary["geometry"]["dimensions"]["collector_width"], .962119)
            self.assertAlmostEqual(summary["geometry"]["dimensions"]["collector_depth"], .408606)
            for case in summary["cases"]:
                a = read_json(output / f"A-{case['case']}.json")
                b = read_json(output / f"B-{case['case']}.json")
                self.assertEqual(a["initial"], b["initial"])
                self.assertEqual(a["geometry"], b["geometry"])
                self.assertEqual(b["result"]["status"], "success")
                final = replay(restore_scene(b["initial"]), b["result"]["goals"])
                self.assertFalse(final.targets)
                self.assertIsNone(final.failure)

    def test_duplicate_seeds_are_rejected(self):
        completed = subprocess.run([sys.executable, "-m", "route_plan_planner.cli", "compare",
                                    "--seed-list", "7,7", "--random-only"], capture_output=True, text=True)
        self.assertEqual(completed.returncode, 2)
        self.assertIn("distinct", completed.stderr)


if __name__ == "__main__":
    unittest.main()
