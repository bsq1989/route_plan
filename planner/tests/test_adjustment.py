import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

from route_plan_planner import PlanningRequest, Target, plan_pickup_adjustment, replay
from route_plan_planner.api import _scene


class PickupAdjustmentTest(unittest.TestCase):
    def request(self, *targets):
        return PlanningRequest(12., 9., (3., 4.5, 0.), targets, timeout=.5)

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
            (Target("item", 3.40, 4.5), ["item"], "unsafe_start"),
            (Target("item", 4.25, 4.5), ["item"], "not_found"),
        ):
            with self.subTest(status=status):
                result = plan_pickup_adjustment(self.request(target), ids)
                self.assertEqual(result.status, status, result.reason)
                self.assertFalse(result.goals)
                self.assertFalse(result.pickup_stops)

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
