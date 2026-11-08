from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from route_plan_planner import PlanningRequest, Target, plan
from route_plan_planner.api import _scene
from route_plan_planner.motion import replay


class PublicApiTest(unittest.TestCase):
    def request(self, **changes):
        values = dict(width=12, height=9, pose=(3, 4.5, 0),
                      targets=[Target("item-a", 5, 4.5), Target(205, 6.5, 5)],
                      method="feasibility", timeout=2, use_ompl=False,
                      request_id="request-001", frame_id="map")
        values.update(changes)
        return PlanningRequest(**values)

    def test_complete_plan_preserves_ids_and_request_and_requires_no_viewer(self):
        request = self.request()
        before = request.to_dict()
        result = plan(request)
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.request_id, "request-001")
        self.assertEqual(result.frame_id, "map")
        self.assertEqual(set(result.expected_collected_ids), {"item-a", 205})
        self.assertEqual(result.remaining_ids, [])
        self.assertEqual(len(result.steps), len(result.goals))
        self.assertEqual(request.to_dict(), before)
        scene, ids, _ = _scene(request)
        trace = replay(scene, result.goals)
        self.assertFalse(trace.targets)
        self.assertIsNone(trace.failure)
        expected = {key for step in result.steps for key in step.expected_collected_ids}
        self.assertEqual(expected, {"item-a", 205})

    def test_dict_input_and_json_output_are_portable(self):
        payload = self.request().to_dict()
        before = deepcopy(payload)
        result = plan(payload)
        decoded = json.loads(json.dumps(result.to_dict(), allow_nan=False))
        self.assertTrue(decoded["success"])
        self.assertEqual(decoded["resolved_request"]["robot_config"]["collector"]["width"], .35)
        repeated = plan(decoded["resolved_request"])
        self.assertTrue(repeated.success)
        self.assertEqual(set(repeated.expected_collected_ids), {"item-a", 205})
        self.assertEqual(payload, before)

    def test_snapshot_does_not_depend_on_robot_file_after_call(self):
        default = plan(self.request()).resolved_request["robot_config"]
        with TemporaryDirectory() as directory:
            path = Path(directory) / "robot.yaml"
            path.write_text(json.dumps(default))
            result = plan(self.request(robot_config=path))
        self.assertIsInstance(result.resolved_request["robot_config"], dict)
        self.assertTrue(plan(result.resolved_request).success)

    def test_initial_contact_is_explicit_and_includes_all_input_ids(self):
        request = self.request(targets=[Target("already-touching", 3.5, 4.5)])
        result = plan(request)
        self.assertTrue(result.success)
        self.assertEqual(result.initially_collected_ids, ["already-touching"])
        self.assertEqual(result.expected_collected_ids, ["already-touching"])
        self.assertEqual(result.goals, [])
        self.assertEqual(result.length, 0.)

    def test_empty_task(self):
        result = plan(self.request(targets=[]))
        self.assertTrue(result.success)
        self.assertEqual(result.length, 0.)
        self.assertEqual(result.goals, [])

    def test_search_failure_does_not_expose_partial_goals(self):
        result = plan(self.request(timeout=1e-12))
        self.assertFalse(result.success)
        self.assertEqual(result.status, "budget_exhausted")
        self.assertEqual(result.goals, [])
        self.assertEqual(result.steps, [])
        self.assertIsNone(result.length)
        self.assertEqual(result.remaining_ids, ["item-a", 205])

    def test_duplicate_ids_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "unique"):
            self.request(targets=[Target("same", 5, 4.5), Target("same", 6, 5)])

    def test_overlapping_objects_share_one_collection_step_and_keep_all_ids(self):
        for method in ("feasibility", "length"):
            for distance in (0., .02, .08):
                with self.subTest(method=method, distance=distance):
                    request = self.request(method=method, timeout=.5,
                                           targets=[Target("a", 5, 4.5), Target(205, 5 + distance, 4.5)])
                    result = plan(request)
                    self.assertTrue(result.success, result.reason)
                    events = [set(step.expected_collected_ids) for step in result.steps if step.expected_collected_ids]
                    self.assertEqual(events, [{"a", 205}])
                    self.assertEqual(set(result.expected_collected_ids), {"a", 205})
                    self.assertTrue(plan(result.resolved_request).success)

    def test_overlapping_chain_at_initial_contact_is_order_independent(self):
        targets = [Target("a", 3.5, 4.5), Target("b", 3.58, 4.5), Target("c", 3.66, 4.5)]
        for sequence in (targets, list(reversed(targets))):
            result = plan(self.request(targets=sequence))
            self.assertTrue(result.success)
            self.assertEqual(set(result.initially_collected_ids), {"a", "b", "c"})
            self.assertEqual(result.goals, [])

    def test_group_member_inside_body_or_outside_map_is_still_invalid(self):
        for targets in ([Target("tool", 3.44, 4.5), Target("body", 3.39, 4.5)],
                        [Target("inside", .12, 2), Target("outside", .04, 2)]):
            with self.subTest(targets=targets), self.assertRaises(ValueError):
                plan(self.request(targets=targets))

    def test_tangent_objects_and_initial_contact_keep_both_ids(self):
        result = plan(self.request(targets=[Target("a", 3.6, 4.5), Target("b", 3.7, 4.5)]))
        self.assertTrue(result.success)
        self.assertEqual(result.initially_collected_ids, ["a"])
        self.assertEqual(set(result.expected_collected_ids), {"a", "b"})

    def test_invalid_poses_and_target_locations_are_rejected(self):
        for changes in ({"pose": (0, 0, 0)}, {"targets": [Target("bad", .02, 4)]},
                        {"targets": [Target("bad", 3, 4.5)]}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                plan(self.request(**changes))

    def test_invalid_request_fields_and_numbers(self):
        for changes in ({"width": float("nan")}, {"timeout": 0}, {"method": "unknown"},
                        {"pose": [1, 2]}, {"seed": -1}, {"schema_version": 1.0},
                        {"frame_id": ""}, {"target_diameter": -1}, {"use_ompl": 1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.request(**changes)
        with self.assertRaises(ValueError):
            Target(True, 5, 4)
        with self.assertRaises(ValueError):
            PlanningRequest.from_dict({**self.request().to_dict(), "typo": 1})

    def test_length_method_and_custom_target_diameter(self):
        result = plan(self.request(method="length", timeout=.5, target_diameter=.16))
        self.assertTrue(result.success)
        self.assertIsNotNone(result.optimization)
        self.assertEqual(result.resolved_request["target_diameter"], .16)


if __name__ == "__main__":
    unittest.main()
