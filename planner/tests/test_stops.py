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
from route_plan_planner.stops import at_stop_scene, center_contact_scene, materialize_stops


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
            self.assertTrue(probe.collector.contains(*probe.local_point(*point)))
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
        self.assertAlmostEqual(scene.pose[0], 5 - scene.safety_body.xmax - scene.target_radius)
        with self.assertRaises(ValueError):
            scene.collect_at_stop()

    def test_initial_tool_coverage_is_one_explicit_stop_not_zero(self):
        result = plan(dict(width=12, height=9, pose=[3, 4.5, 0],
                           targets=[dict(id="initial", x=3.55, y=4.5)], method="stops", timeout=.5))
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.initially_collected_ids, [])
        self.assertEqual(result.stop_count, 1)
        self.assertEqual(result.length, 0.)
        self.assertEqual(result.pickup_stops[0].target_ids, ("initial",))

    def test_expired_optimization_keeps_safe_materialized_seed(self):
        scene = self.scene([(5, 4.35), (5.1, 4.5), (5.2, 4.65)])
        seed = StaticPlanner(PlannerOptions(ompl=False)).plan(center_contact_scene(scene))
        expected = materialize_stops(scene, seed)
        seed.elapsed = 100.
        result = StopsPlanner(PlannerOptions(timeout=.1, ompl=False)).plan(scene, initial=seed)
        self.assertEqual(result.status, "success", result.reason)
        self.assertEqual(result.goals, expected.goals)
        self.assertEqual(result.pickup_stops, expected.pickup_stops)

    def test_rotation_contact_materializes_at_analytic_pose(self):
        scene = self.scene([(3.45, 4.8)])
        seed = StaticPlanner(PlannerOptions(ompl=False)).plan(center_contact_scene(scene))
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
        self.assertEqual(restored.pickup_rule, "center")
        result = StopsPlanner(PlannerOptions(timeout=.5, ompl=False)).plan(restored)
        final = replay(restored, result.goals, pickup_stops=result.pickup_stops)
        self.assertFalse(final.targets)
        self.assertIsNone(final.failure)

    def test_circle_tangency_and_partial_overlap_do_not_count_as_coverage(self):
        for dx, dy in ((.05, 0), (.02, 0), (.03, .025)):
            with self.subTest(dx=dx, dy=dy):
                scene = at_stop_scene(self.scene([]))
                point = (scene.pose[0] + scene.collector.xmax + dx,
                         scene.pose[1] + (scene.collector.ymax + dy if dy else 0))
                self.assertTrue(scene.collector.intersects_circle(*scene.local_point(*point), scene.target_radius))
                scene.add_target(*point)
                self.assertEqual(scene.collect_at_stop(), ())
                self.assertEqual(len(scene.targets), 1)

    def test_overlapping_member_with_external_center_is_not_cleared(self):
        scene = at_stop_scene(self.scene([]))
        ids = scene.add_targets([(3 + scene.collector.xmax - .01, 4.5),
                                 (3 + scene.collector.xmax + .02, 4.5)])
        self.assertEqual(scene.target_groups, (ids,))
        self.assertEqual(scene.collect_at_stop(), (ids[0],))
        self.assertEqual(set(scene.targets), {ids[1]})

    def test_api_overlapping_batch_changes_pose_until_all_centers_fit(self):
        targets = [dict(id="inside", x=3.58, y=4.5), dict(id="outside", x=3.61, y=4.5)]
        result = plan(dict(width=12, height=9, pose=[3, 4.5, 0], targets=targets,
                           method="stops", timeout=1., use_ompl=False))
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.stop_count, 1)
        self.assertNotEqual(result.pickup_stops[0].pose, (3, 4.5, 0))
        probe = at_stop_scene(self.scene([]))
        probe.pose = result.pickup_stops[0].pose
        for target in targets:
            self.assertTrue(probe.collector.contains(*probe.local_point(target["x"], target["y"])))

    def test_center_inside_is_enough_without_containing_the_entire_disk(self):
        scene = at_stop_scene(self.scene([]))
        ids = scene.add_targets([(3 + scene.collector.xmax - .01, 4.5)])
        self.assertEqual(scene.collect_at_stop(), ids)
        self.assertFalse(scene.targets)

    def test_rotated_stop_uses_body_frame_center_coordinates(self):
        scene = at_stop_scene(self.scene([]))
        yaw = .7
        scene.pose = (3, 4.5, yaw)
        u, v = (scene.collector.xmin + scene.collector.xmax) / 2, .1
        ids = scene.add_targets([(3 + math.cos(yaw) * u - math.sin(yaw) * v,
                                 4.5 + math.sin(yaw) * u + math.cos(yaw) * v)])
        self.assertEqual(scene.collect_at_stop(), ids)

    def test_legacy_contact_stop_archive_keeps_its_old_rule(self):
        scene = at_stop_scene(self.scene([]))
        scene.add_target(3 + scene.collector.xmax + .05, 4.5)
        record = json.loads(json.dumps(describe_scene(scene)))
        record["world_config"]["custom"].pop("pickup_rule")
        legacy = restore_scene(record)
        self.assertEqual(legacy.pickup_rule, "contact")
        self.assertEqual(len(legacy.collect_at_stop()), 1)

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
