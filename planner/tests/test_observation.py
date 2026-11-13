import json
import math
import unittest

from route_plan_planner import plan_observation_step


class ObservationTests(unittest.TestCase):
    request = {"pose": [0., 0., 0.], "targets": [], "fence_enabled": False, "use_ompl": False}
    coarse = [{"id": "far", "x": 1.85, "y": 0., "error_radius": .15}]

    def test_remeasures_between_hops_and_rejects_stale_continuation(self):
        turn = math.pi / 2
        route = [[[0., 0., turn], [0., .30, turn]], [[0., .60, turn]]]
        first = plan_observation_step(self.request, self.coarse, pending_hops=route).to_dict()
        self.assertEqual(first["status"], "observe")
        self.assertEqual(first["goals"], route[0])
        self.assertEqual(first["pending_hops"], route[1:])
        self.assertEqual(first["observation_pose"], route[1][-1])

        request = {**self.request, "pose": first["goals"][-1]}
        updated = [*self.coarse, {"id": "new", "x": 0., "y": .95, "error_radius": .10}]
        second = plan_observation_step(request, updated, pending_hops=first["pending_hops"]).to_dict()
        self.assertNotEqual(second["goals"], route[1])
        self.assertIn("pending_uncertain_clearance", second["rejections"])
        json.dumps(second, allow_nan=False)

    def test_coarse_intrusion_requires_independent_start_clearance(self):
        coarse = [{"id": "side", "x": .20, "y": .45, "error_radius": .14}]
        blocked = plan_observation_step(self.request, coarse)
        self.assertEqual(blocked.status, "blocked")
        self.assertFalse(blocked.goals)
        result = plan_observation_step(self.request, coarse, initial_clearance_confirmed=True)
        self.assertEqual(result.status, "observe")
        x, y, yaw = result.goals[0]
        self.assertGreater(x, .05)
        self.assertAlmostEqual(y, 0.)
        self.assertAlmostEqual(yaw, 0.)

    def test_confirmed_target_supersedes_coarse_hint_for_pickup(self):
        request = {**self.request, "targets": [{"id": "far", "x": .9, "y": 0.}]}
        coarse = [{"id": "far", "x": .3, "y": .4, "error_radius": .5}]
        response = plan_observation_step(request, coarse)
        self.assertEqual(response.status, "pickup")
        self.assertEqual(response.pickup_stops[0].target_ids, ("far",))
        self.assertFalse(response.pending_hops)
