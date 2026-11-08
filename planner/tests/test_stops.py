import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

from route_plan_sim import Scene
from route_plan_planner import PlannerOptions, StaticPlanner, StopsPlanner, plan
from route_plan_planner.motion import replay
from route_plan_planner.records import describe_scene, restore_scene
from route_plan_planner.stops import at_stop_scene, materialize_stops


class PickupStopsTest(unittest.TestCase):
    def scene(self, points):
        scene = Scene()
        scene.targets.clear()
        scene.add_targets(points)
        return scene

    def test_nonoverlapping_cluster_is_covered_at_one_pose(self):
        scene = self.scene([(5, 4.35), (5.1, 4.5), (5.2, 4.65)])
        self.assertTrue(all(len(group) == 1 for group in scene.target_groups))
        result = StopsPlanner(PlannerOptions(timeout=1, ompl=False)).plan(scene)
        self.assertEqual(result.status, "success", result.reason)
        self.assertEqual(len(result.pickup_stops), 1)
        self.assertEqual(set(result.pickup_stops[0]["target_ids"]), set(scene.targets))
        probe = at_stop_scene(scene)
        probe.pose = result.pickup_stops[0]["pose"]
        for point in probe.targets.values():
            self.assertTrue(probe.collector.intersects_circle(*probe.local_point(*point), probe.target_radius))
            self.assertFalse(probe.safety_body.intersects_circle(*probe.local_point(*point), probe.target_radius))

    def test_crossing_separated_objects_is_not_counted_as_one_stop(self):
        scene = self.scene([(5, 4.5), (6, 4.5), (7, 4.5)])
        result = StopsPlanner(PlannerOptions(timeout=.5, ompl=False)).plan(scene)
        self.assertEqual(result.status, "success", result.reason)
        self.assertEqual(len(result.pickup_stops), 3)
        for dt in (.013, .7, 2.):
            final = replay(at_stop_scene(scene), result.goals, dt, result.pickup_stops)
            self.assertIsNone(final.failure)
            self.assertFalse(final.targets)
            self.assertEqual(set(final.collected), set(scene.targets))

    def test_targets_are_obstacles_until_explicit_pickup(self):
        scene = at_stop_scene(self.scene([(5, 4.5)]))
        scene.forward(4)
        self.assertIsNotNone(scene.failure)
        self.assertFalse(scene.collected)
        with self.assertRaises(ValueError):
            scene.collect_at_stop()

    def test_initial_tool_coverage_is_one_explicit_stop_not_zero(self):
        result = plan(dict(width=12, height=9, pose=[3, 4.5, 0],
                           targets=[dict(id="initial", x=3.6, y=4.5)], method="stops", timeout=.5))
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.initially_collected_ids, [])
        self.assertEqual(result.stop_count, 1)
        self.assertEqual(result.length, 0.)
        self.assertEqual(result.pickup_stops[0].target_ids, ("initial",))

    def test_expired_optimization_keeps_safe_materialized_seed(self):
        scene = self.scene([(5, 4.35), (5.1, 4.5), (5.2, 4.65)])
        seed = StaticPlanner(PlannerOptions(ompl=False)).plan(scene)
        expected = materialize_stops(scene, seed)
        seed.elapsed = 100.
        result = StopsPlanner(PlannerOptions(timeout=.1, ompl=False)).plan(scene, initial=seed)
        self.assertEqual(result.status, "success", result.reason)
        self.assertEqual(result.goals, expected.goals)
        self.assertEqual(result.pickup_stops, expected.pickup_stops)

    def test_rotation_contact_materializes_at_analytic_pose(self):
        scene = self.scene([(3.45, 4.8)])
        seed = StaticPlanner(PlannerOptions(ompl=False)).plan(scene)
        self.assertEqual(seed.status, "success")
        materialized = materialize_stops(scene, seed)
        self.assertEqual(len(materialized.pickup_stops), 1)
        final = replay(at_stop_scene(scene), materialized.goals, dt=.013,
                       pickup_stops=materialized.pickup_stops)
        self.assertIsNone(final.failure)
        self.assertFalse(final.targets)

    def test_api_returns_portable_stop_checkpoints_and_all_original_ids(self):
        request = dict(width=12, height=9, pose=[3, 4.5, 0], method="stops", timeout=1, use_ompl=False,
                       targets=[dict(id="a", x=5, y=4.35), dict(id=205, x=5.1, y=4.5),
                                dict(id="c", x=5.2, y=4.65)])
        result = plan(request)
        self.assertTrue(result.success, result.reason)
        payload = json.loads(json.dumps(result.to_dict(), allow_nan=False))
        self.assertEqual(payload["stop_count"], 1)
        self.assertEqual(set(payload["pickup_stops"][0]["target_ids"]), {"a", 205, "c"})
        index = payload["pickup_stops"][0]["goal_index"]
        self.assertEqual(payload["pickup_stops"][0]["pose"], payload["goals"][index])
        self.assertEqual(set(payload["steps"][index]["expected_collected_ids"]), {"a", 205, "c"})
        self.assertTrue(plan(result.resolved_request).success)

    def test_archive_roundtrip_retains_stop_collection_policy(self):
        scene = at_stop_scene(self.scene([(5, 4.35), (5.1, 4.5)]))
        restored = restore_scene(json.loads(json.dumps(describe_scene(scene))))
        self.assertEqual(restored.collection_mode, "at_stop")
        result = StopsPlanner(PlannerOptions(timeout=.5, ompl=False)).plan(restored)
        final = replay(restored, result.goals, pickup_stops=result.pickup_stops)
        self.assertFalse(final.targets)
        self.assertIsNone(final.failure)

    def test_invalid_pickup_checkpoint_is_rejected(self):
        scene = at_stop_scene(self.scene([(5, 4.5)]))
        with self.assertRaisesRegex(ValueError, "indices"):
            replay(scene, [], pickup_stops=[dict(goal_index=0, target_ids=[11])])
        with self.assertRaisesRegex(ValueError, "coverage"):
            replay(scene, [scene.pose], pickup_stops=[dict(goal_index=0, target_ids=[999])])

    def test_cli_stop_mode_can_be_replayed_without_replanning(self):
        with TemporaryDirectory() as folder:
            record = Path(folder) / "plan.json"
            solved = subprocess.run([sys.executable, "-m", "route_plan_planner.cli", "solve", "--method", "stops",
                                     "--timeout", "1", "--no-ompl", "--output", str(record)],
                                    capture_output=True, text=True)
            self.assertEqual(solved.returncode, 0, solved.stderr)
            played = subprocess.run([sys.executable, "-m", "route_plan_planner.cli", "replay", str(record)],
                                    capture_output=True, text=True)
            self.assertEqual(played.returncode, 0, played.stderr)


if __name__ == "__main__":
    unittest.main()
