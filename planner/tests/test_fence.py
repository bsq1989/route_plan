import json
import math
import unittest

from route_plan_planner.api import PlanningRequest, _scene, plan
from route_plan_planner.motion import replay
from route_plan_planner.records import describe_scene, restore_scene


# A rectangular notch joins the top boundary and separates the upper arms.
NOTCH = [(0, 0), (6, 0), (6, 5), (3.2, 5), (3.2, 2.7), (2.8, 2.7), (2.8, 5), (0, 5)]


class FenceTest(unittest.TestCase):
    def request(self, **overrides):
        return {"fence": NOTCH, "pose": [1, 3.6, 0], "targets": [],
                "method": "stops", "use_ompl": False, **overrides}

    def scene(self, **overrides):
        return _scene(PlanningRequest.from_dict(self.request(**overrides)))[0]

    def test_map_coordinates_and_fence_survive_planning_and_replay(self):
        fence = [[-3, -2], [1, -2], [2, 0], [1, 2], [-3, 2], [-3, -2]]
        request = self.request(fence=fence, pose=[-2, 0, 0],
                               targets=[{"id": "cup", "x": -.3, "y": 0}])
        result = plan(request)
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.expected_collected_ids, ["cup"])
        self.assertLess(result.final_pose[0], 0)
        resolved = json.loads(json.dumps(result.to_dict()))["resolved_request"]
        self.assertEqual(resolved["fence"], fence[:-1])
        scene = self.scene(**resolved)
        archived = restore_scene(json.loads(json.dumps(describe_scene(scene))))
        stops = [{"goal_index": stop.goal_index, "target_ids": [1]} for stop in result.pickup_stops]
        final = replay(archived, result.goals, pickup_stops=stops, minimum_motion=True)
        self.assertIsNone(final.failure)
        self.assertFalse(final.targets)

    def test_concave_notch_blocks_translation_between_legal_endpoints(self):
        scene = self.scene()
        self.scene(pose=[5, 3.6, 0])  # The destination by itself fits.
        scene.forward(4)
        self.assertEqual(scene.failure, "Map boundary")
        self.assertAlmostEqual(scene.pose[0], 2.8 - scene.collector.xmax)

    def test_concave_notch_can_cross_footprint_with_all_corners_inside(self):
        with self.assertRaisesRegex(ValueError, "fit strictly inside"):
            self.scene(pose=[3, 2.95, math.pi])

    def test_fence_vertex_can_hit_moving_rectangle_between_its_corners(self):
        fence = [(0, 0), (6, 0), (6, 5), (3.2, 5), (3, 2.6), (2.8, 5), (0, 5)]
        scene = self.scene(fence=fence, pose=[3, 1.4, math.pi / 2])
        scene.forward(2)
        self.assertEqual(scene.failure, "Map boundary")
        self.assertAlmostEqual(scene.pose[1], 2.6 - scene.collector.xmax)

    def test_rotation_sweep_hits_fence_even_when_endpoints_fit(self):
        fence = [(0, 0), (4, 0), (4, 4), (0, 4)]
        scene = self.scene(fence=fence, pose=[.56, 2, math.pi / 2])
        self.scene(fence=fence, pose=[.56, 2, -math.pi / 2])
        scene.advance(rotation=math.pi)
        self.assertEqual(scene.failure, "Map boundary")
        self.assertGreater(scene.pose[2], math.pi / 2)
        self.assertLess(scene.pose[2], math.pi)

    def test_disk_tangent_to_diagonal_boundary_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Target must be inside"):
            self.scene(fence=[(0, 0), (4, 0), (0, 4)], pose=[1, 1, 0],
                       targets=[{"id": 1, "x": 2., "y": 2. - .05 * math.sqrt(2)}])

    def test_pickup_near_acute_corner_finds_a_narrow_approach(self):
        fence = [[.662275845, -2.927050271], [-.837475480, .365602301],
                 [1.607158116, 1.497195037], [2.675180813, -.846052098],
                 [2.221171244, -1.054056482], [2.660034521, -2.016756562]]
        request = self.request(fence=fence, pose=[.519444754, -.462152687, -2.468598640],
                               targets=[{"id": "corner", "x": -.697207904, "y": .350987492}])
        response = plan(request)
        self.assertTrue(response.success, response.reason)
        self.assertEqual(response.expected_collected_ids, ["corner"])
        self.assertEqual(response.stop_count, 1)

    def test_invalid_fence_rejected(self):
        for fence in ([], [(0, 0), (1, 1), (0, 1), (1, 0)],
                      [(0, 0), (1, 0), (2, 0)], [(0, 0), (1, 0), (1, math.inf)],
                      [(0, 0), (2, 0), (1, 0), (2, 1), (0, 1)]):
            with self.subTest(fence=fence), self.assertRaisesRegex(ValueError, "fence"):
                PlanningRequest.from_dict(self.request(fence=fence))
