import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

from route_plan_planner import PlanningRequest, Target, plan_pickup_adjustment, replay
from route_plan_planner.adjustment import _simulation_requests, _translate_pose
from route_plan_planner.api import _scene

RECTANGULAR_ROBOT = Path(__file__).resolve().parents[2] / "sim/tests/fixtures/robot-rectangular.yaml"

class PickupAdjustmentTest(unittest.TestCase):
    def request(self, *targets):
        return PlanningRequest(12., 9., (3., 4.5, 0.), targets, timeout=.5, robot_config=RECTANGULAR_ROBOT)

    def verify(self, result, picked, pending, limit=.40):
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.stop_count, 1)
        self.assertEqual(set(result.expected_collected_ids), set(picked))
        self.assertEqual(set(result.remaining_ids), set(pending))
        scene, ids, _ = _scene(PlanningRequest.from_dict(result.resolved_request))
        self.assertTrue(all(math.dist(scene.pose[:2], goal[:2]) <= limit + 1e-9 for goal in result.goals))
        stop = result.pickup_stops[0]
        self.assertEqual(stop.goal_index, len(result.goals) - 1)
        keys = [key for key, name in ids.items() if name in stop.target_ids]
        for dt in (.013, .7):
            end = replay(scene, result.goals, dt, [{"goal_index": stop.goal_index, "target_ids": keys}],
                         minimum_motion=True)
            self.assertIsNone(end.failure)
            self.assertEqual({ids[key] for key in end.targets}, set(pending))

    def test_forward_and_small_error_are_corrected_with_legal_moves(self):
        for x in (3.75, 3.64):
            with self.subTest(x=x):
                request = self.request(Target("item", x, 4.5), Target(2, 5., 5.))
                result = plan_pickup_adjustment(request, ["item"])
                self.verify(result, ["item"], [2])
                self.assertGreater(result.length, .05)
                self.assertEqual(request.method, "length")

    def test_lateral_error_can_be_corrected_without_translation(self):
        request = self.request(Target("item", 3.60, 4.68))
        result = plan_pickup_adjustment(request, ["item"], max_displacement=0.)
        self.verify(result, ["item"], [], limit=0.)
        self.assertGreater(abs(result.final_pose[2]), math.radians(1.))

    def test_polygon_adjustment_prefers_clear_stop_with_in_place_turn(self):
        request = dict(pose=(0., 0., 0.), mode="auto_fence", timeout=1.,
                       fence=[[-1, -1], [1.5, -1], [1.5, 1.5], [-1, 1.5]],
                       targets=[dict(id="item", x=.35, y=-.46), dict(id="neighbor", x=1.1, y=-.4)])
        result = plan_pickup_adjustment(request, ["item"], max_displacement=0.)
        self.verify(result, ["item"], ["neighbor"], limit=0.)
        self.assertGreaterEqual(result.metrics["min_pickup_clearance_m"], .03)
        self.assertGreaterEqual(result.metrics["min_near_edge_clearance_m"], .03)

    def test_polygon_adjustment_prioritizes_the_center_when_moving(self):
        request = dict(pose=(0., 0., 0.), mode="auto_fence", timeout=1.,
                       targets=[dict(id="item", x=.60, y=-.42)])
        result = plan_pickup_adjustment(request, ["item"])
        self.verify(result, ["item"], [])
        self.assertEqual(result.metrics["center_pickup_count"], 1)

    def test_already_covered_batch_is_picked_at_the_measured_pose(self):
        request = self.request(Target("a", 3.58, 4.45), Target("b", 3.60, 4.54))
        result = plan_pickup_adjustment(request, ["a", "b"])
        self.verify(result, ["a", "b"], [])
        self.assertEqual(result.goals, [request.pose])
        self.assertEqual(result.length, 0.)

    def test_nearby_item_blocks_straight_approach_but_is_kept_in_scene(self):
        request = self.request(Target("item", 3.75, 4.5), Target("neighbor", 3.54, 4.27))
        result = plan_pickup_adjustment(request, ["item"])
        self.verify(result, ["item"], ["neighbor"])
        self.assertNotAlmostEqual(result.final_pose[1], request.pose[1])

    def test_failures_return_no_movement(self):
        for target, ids, status in (
            (Target("item", 3.75, 4.5), ["missing"], "target_not_observed"),
            (Target("item", 3.40, 4.5), ["item"], "body_contact"),
            (Target("item", 3.44, 4.5), ["item"], "not_found"),
            (Target("item", 4.25, 4.5), ["item"], "not_found"),
        ):
            with self.subTest(status=status):
                result = plan_pickup_adjustment(self.request(target), ids)
                self.assertEqual(result.status, status, result.reason)
                self.assertFalse(result.goals)
                self.assertFalse(result.pickup_stops)

    def test_recovery_clears_safety_overlap_before_pickup(self):
        request = dict(pose=(0., 0., 0.), timeout=.5, targets=[dict(id="item", x=.26, y=.36)])
        result = plan_pickup_adjustment(request, ["item"])
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.expected_collected_ids, ["item"])
        count = result.metrics["escape_goal_count"]
        self.assertGreater(count, 0)
        self.assertLess(count, len(result.goals))
        public, simulation = _simulation_requests(result.resolved_request, .40)
        scene, ids, _ = _scene(simulation, allow_safety_intrusions=True)
        self.assertTrue(scene.recovering)
        with self.assertRaisesRegex(ValueError, "before clearing"):
            scene.collect_at_stop()
        goals = [_translate_pose(goal, public.pose, simulation.pose) for goal in result.goals]
        self.assertTrue(all(math.dist(scene.pose[:2], goal[:2]) <= .40 for goal in goals))
        pickup = {"goal_index": result.pickup_stops[0].goal_index, "target_ids": list(ids)}
        for dt in (.013, .7):
            escaped = replay(scene, goals[:count], dt, minimum_motion=True)
            self.assertIsNone(escaped.failure)
            self.assertFalse(escaped.recovering)
            self.assertEqual(escaped.targets, scene.targets)
            final = replay(scene, goals, dt, pickup_stops=[pickup], minimum_motion=True)
            self.assertIsNone(final.failure)
            self.assertFalse(final.targets)

    def test_recovery_respects_displacement_and_normal_safety_after_exit(self):
        request = self.request(Target("item", 2.58, 4.5))
        limited = plan_pickup_adjustment(request, ["item"], max_displacement=.04)
        self.assertEqual(limited.status, "not_found")
        self.assertFalse(limited.goals)

        scene, _, _ = _scene(PlanningRequest.from_dict(dict(request.to_dict(), method="stops")),
                             allow_safety_intrusions=True)
        scene.forward(.30)
        self.assertFalse(scene.recovering)
        scene.advance(rotation=math.pi)
        self.assertIsNone(scene.failure)
        scene.forward(.40)
        self.assertIsNotNone(scene.failure)

    def test_cli_can_replay_recovery_followed_by_pickup(self):
        with TemporaryDirectory() as folder:
            source = Path(folder) / "request.json"
            output = Path(folder) / "adjustment.json"
            source.write_text(json.dumps(dict(pose=[0., 0., 0.], targets=[dict(id=42, x=.37, y=.37)])))
            cli = [sys.executable, "-m", "route_plan_planner.cli"]
            planned = subprocess.run(
                [*cli, "adjust", "--input", str(source), "--target-ids", "[42]", "--output", str(output)],
                capture_output=True, text=True,
            )
            self.assertEqual(planned.returncode, 0, planned.stderr)
            played = subprocess.run([*cli, "replay", str(output)], capture_output=True, text=True)
            self.assertEqual(played.returncode, 0, played.stderr)
            self.assertIn("1 -> 0", played.stdout)

    def test_mapless_request_keeps_fixed_coordinates_and_obstacle_checks(self):
        request = dict(pose=(-1., 0., 0.), frame_id="pickup_origin", timeout=.5,
                       robot_config=str(RECTANGULAR_ROBOT),
                       targets=[dict(id="item", x=-.25, y=0.), dict(id="neighbor", x=-.46, y=-.23)])
        result = plan_pickup_adjustment(request, ["item"])
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.frame_id, request["frame_id"])
        self.assertEqual(result.resolved_request["pose"], list(request["pose"]))
        self.assertNotIn("width", result.resolved_request)
        self.assertNotIn("height", result.resolved_request)
        self.assertEqual(result.expected_collected_ids, ["item"])
        self.assertEqual(result.remaining_ids, ["neighbor"])
        self.assertGreater(result.final_pose[1], 0.)
        self.assertTrue(all(math.dist(request["pose"][:2], goal[:2]) <= .40 for goal in result.goals))
        self.assertEqual(result.pickup_stops[0].pose, result.goals[-1])

        public, simulation = _simulation_requests(result.resolved_request, .40)
        scene, ids, _ = _scene(simulation)
        goals = [_translate_pose(goal, public.pose, simulation.pose) for goal in result.goals]
        pickup = {"goal_index": result.pickup_stops[0].goal_index,
                  "target_ids": [key for key, name in ids.items() if name == "item"]}
        final = replay(scene, goals, dt=.7, pickup_stops=[pickup], minimum_motion=True)
        self.assertIsNone(final.failure)
        self.assertEqual([ids[key] for key in final.targets], ["neighbor"])

    def test_mapless_request_requires_either_both_dimensions_or_neither(self):
        request = dict(pose=(0., 0., 0.), targets=[dict(id="item", x=.58, y=0.)])
        self.assertEqual(plan_pickup_adjustment(request, ["item"]).goals, [request["pose"]])
        with self.assertRaisesRegex(ValueError, "both width and height"):
            plan_pickup_adjustment(dict(request, width=12.), ["item"])

    def test_cli_can_save_and_replay_a_partial_pickup_independently(self):
        demo = Path(__file__).resolve().parents[1] / "examples" / "pickup_adjustment.json"
        with TemporaryDirectory() as folder:
            output = Path(folder) / "adjustment.json"
            cli = [sys.executable, "-m", "route_plan_planner.cli"]
            planned = subprocess.run(
                [*cli, "adjust", "--input", str(demo), "--target-ids", '["item-a"]', "--output", str(output)],
                capture_output=True, text=True,
            )
            self.assertEqual(planned.returncode, 0, planned.stderr)
            response = json.loads(output.read_text())["sdk_response"]
            self.assertEqual(response["expected_collected_ids"], ["item-a"])
            self.assertEqual(response["remaining_ids"], ["nearby-item"])
            played = subprocess.run([*cli, "replay", str(output)], capture_output=True, text=True)
            self.assertEqual(played.returncode, 0, played.stderr)
            self.assertIn("2 -> 1", played.stdout)
