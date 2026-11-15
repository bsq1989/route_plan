import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import yaml

from route_plan_sim import Scene
from route_plan_planner import PlannerOptions, StaticPlanner, StopsPlanner, plan
from route_plan_planner.core import PlanResult
from route_plan_planner.motion import MIN_ROTATION, MIN_TRANSLATION, replay, steer
from route_plan_planner.records import describe_scene, restore_scene
from route_plan_planner.stops import at_stop_scene, center_contact_scene, materialize_stops

PREVIOUS_ROBOT = Path(__file__).resolve().parents[2] / "sim/tests/fixtures/robot-v0.2.4.yaml"


class PickupStopsTest(unittest.TestCase):
    def scene(self, points):
        scene = Scene(robot_config=PREVIOUS_ROBOT)
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

    def test_default_union_covers_both_sides_from_one_stop(self):
        result = plan(dict(pose=[0., 0., 0.], mode="auto_fence", method="stops",
                           targets=[dict(id="left", x=.50, y=.31), dict(id="right", x=.50, y=-.31)],
                           use_ompl=False))
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.goals, [(0., 0., 0.)])
        self.assertEqual(result.stop_count, 1)
        self.assertCountEqual(result.pickup_stops[0].target_ids, ["left", "right"])

    def test_default_union_does_not_collect_inside_safety_margin(self):
        scene = at_stop_scene(Scene(scenario="empty"))
        point = (.30, .38)
        self.assertTrue(scene.collector.contains(*point))
        self.assertFalse(scene.body.intersects_circle(*point, scene.target_radius))
        self.assertTrue(scene.safety_body.intersects_circle(*point, scene.target_radius))
        pose_x, pose_y, _ = scene.pose
        scene.add_targets([(pose_x + point[0], pose_y + point[1])], allow_safety_intrusions=True)
        with self.assertRaisesRegex(ValueError, "before clearing"):
            scene.collect_at_stop()

    def test_chamfer_allows_a_pickup_outside_the_swept_safety_body(self):
        result = plan(dict(pose=[0., 0., 0.], mode="auto_fence", method="stops",
                           targets=[dict(id="left", x=.37, y=.37)], use_ompl=False))
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.goals, [(0., 0., 0.)])
        self.assertEqual(result.pickup_stops[0].target_ids, ("left",))

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
        self.assertEqual([stop["target_ids"] for stop in result.pickup_stops],
                         [stop["target_ids"] for stop in expected.pickup_stops])
        with self.assertRaisesRegex(ValueError, "deadband"):
            replay(at_stop_scene(scene), expected.goals, pickup_stops=expected.pickup_stops, minimum_motion=True)
        final = replay(at_stop_scene(scene), result.goals, pickup_stops=result.pickup_stops, minimum_motion=True)
        self.assertFalse(final.targets)
        self.assertIsNone(final.failure)

    def test_contact_seed_exhaustion_leaves_budget_for_executable_search(self):
        scene = at_stop_scene(self.scene([(5, 4.5), (6, 4.5)]))
        options = PlannerOptions(timeout=1., max_trials=120, max_expansions=30, ompl=False)
        with patch("route_plan_planner.stops.StaticPlanner") as seed_planner:
            def exhaust_seed(_scene):
                seed_options = seed_planner.call_args.args[0]
                self.assertLess(seed_options.timeout, options.timeout)
                return PlanResult("budget_exhausted", elapsed=seed_options.timeout,
                                  trials=seed_options.max_trials, expanded=seed_options.max_expansions)

            seed_planner.return_value.plan.side_effect = exhaust_seed
            result = StopsPlanner(options).plan(scene)

        self.assertEqual(result.status, "success", result.reason)
        self.assertIsNone(result.optimization["baseline_stop_count"])
        self.assertEqual(result.optimization["seed_status"], "budget_exhausted")
        final = replay(scene, result.goals, pickup_stops=result.pickup_stops, minimum_motion=True)
        self.assertFalse(final.targets)
        self.assertIsNone(final.failure)

    def test_replay_rejects_small_translation_and_each_small_turn(self):
        scene = at_stop_scene(self.scene([]))
        x, y, yaw = scene.pose
        for goal in ((x + MIN_TRANSLATION, y, yaw), (x, y, yaw - MIN_ROTATION),
                     (x + 1, y + .01, yaw + .1), (x + 1, y, yaw + MIN_ROTATION)):
            with self.subTest(goal=goal), self.assertRaisesRegex(ValueError, "deadband"):
                replay(scene, [goal], minimum_motion=True)

    def test_small_turn_can_be_reached_with_two_legal_turns(self):
        scene = at_stop_scene(self.scene([]))
        goal = (*scene.pose[:2], scene.pose[2] - MIN_ROTATION)
        transition = steer(scene, goal, minimum_motion=True)
        self.assertIsNotNone(transition)
        final = replay(scene, transition.goals, minimum_motion=True)
        self.assertAlmostEqual(final.pose[2], goal[2])
        self.assertIsNone(final.failure)

    def test_nearby_pickup_replans_to_a_legal_move(self):
        scene = at_stop_scene(self.scene([]))
        scene.add_target(3.62, 4.5)
        result = StopsPlanner(PlannerOptions(timeout=1., ompl=False)).plan(scene)
        self.assertEqual(result.status, "success", result.reason)
        self.assertEqual(result.optimization["objective"], ["pickup_stop_count"])
        self.assertEqual(len(result.pickup_stops), 1)
        self.assertGreater(result.length, MIN_TRANSLATION)
        self.assertLess(result.length, .1)
        final = replay(scene, result.goals, pickup_stops=result.pickup_stops, minimum_motion=True)
        self.assertIsNone(final.failure)
        self.assertFalse(final.targets)

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
                       robot_config=PREVIOUS_ROBOT, pickup_margin_m=0.,
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

    def test_archive_replay_uses_the_recorded_mounting_rule(self):
        record = describe_scene(at_stop_scene(self.scene([])))
        record["vehicle_config"]["collector"]["gap"] = .23
        for version, expected_gap in ((None, .23), (2, .10), (3, .23)):
            with self.subTest(version=version):
                archive = json.loads(json.dumps(record))
                custom = archive["world_config"]["custom"]
                if version is None:
                    del custom["rule_version"]
                else:
                    custom["rule_version"] = version
                restored = restore_scene(archive)
                self.assertAlmostEqual(restored.collector_gap, expected_gap)
                self.assertAlmostEqual(restored.collector.xmin, .25 + expected_gap)
                self.assertAlmostEqual(describe_scene(restored)["vehicle_config"]["collector"]["gap"], expected_gap)

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
                           method="stops", timeout=1., use_ompl=False, robot_config=PREVIOUS_ROBOT))
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.stop_count, 1)
        self.assertNotEqual(result.pickup_stops[0].pose, (3, 4.5, 0))
        probe = at_stop_scene(self.scene([]))
        probe.pose = result.pickup_stops[0].pose
        for target in targets:
            self.assertTrue(probe.collector.contains(*probe.local_point(target["x"], target["y"])))

    def test_pickup_inset_splits_unfit_batch_with_both_fences_enabled(self):
        robot = yaml.safe_load(PREVIOUS_ROBOT.read_text())
        robot["collector"].update(width=.19, depth=.09, gap=.183)
        targets = [dict(id="a", x=4.2, y=4.425), dict(id="b", x=4.2, y=4.575)]
        request = dict(pose=[3, 4.5, 0], targets=targets, method="stops", robot_config=robot,
                       fence=[[2, 3], [5, 3], [5, 6], [2, 6]], mode="auto_fence",
                       timeout=2., use_ompl=False)
        nominal = plan(dict(request, pickup_margin_m=0.))
        self.assertTrue(nominal.success, nominal.reason)
        self.assertEqual(nominal.stop_count, 1)
        self.assertEqual(nominal.resolved_request["pickup_margin_m"], 0.)
        inset = plan(request)
        self.assertTrue(inset.success, inset.reason)
        self.assertEqual(inset.stop_count, 2)
        self.assertCountEqual(inset.expected_collected_ids, ("a", "b"))
        self.assertEqual(inset.resolved_request["pickup_margin_m"], .03)
        self.assertEqual(inset.resolved_request["robot_config"]["collector"], robot["collector"])
        impossible = plan(dict(request, pickup_margin_m=.05))
        self.assertFalse(impossible.success)
        self.assertEqual(impossible.goals, [])

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
