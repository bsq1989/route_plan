import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import yaml

from route_plan_sim import Scene
from route_plan_planner.api import PlanningRequest, _scene, plan
from route_plan_planner.adjustment import plan_pickup_adjustment
from route_plan_planner.escape import plan_escape
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

    def test_disabled_fence_plans_in_map_coordinates_without_dimensions(self):
        request = self.request(fence=[[-4, -3], [-1, -3], [-1, -1], [-4, -1]],
                               pose=[-2., -2., 0.], targets=[{"id": "outside", "x": -.5, "y": -2.}])
        with self.assertRaisesRegex(ValueError, "Target must be inside"):
            plan(request)
        result = plan(PlanningRequest.from_dict({**request, "fence_enabled": False}))
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.expected_collected_ids, ["outside"])
        self.assertLess(result.final_pose[0], -1.)
        resolved = json.loads(json.dumps(result.resolved_request))
        self.assertFalse(resolved["fence_enabled"])
        self.assertEqual(resolved["fence"], request["fence"])
        scene, _, _ = _scene(PlanningRequest.from_dict(resolved))
        restored = restore_scene(describe_scene(scene))
        self.assertIsNone(restored.fence)
        self.assertEqual(restored.map_bounds, scene.map_bounds)
        self.assertEqual(restored.pose, tuple(request["pose"]))

    def test_disabled_fence_preserves_explicit_rectangle(self):
        request = self.request(fence_enabled=False, width=4, height=4, pose=[1, 1, 0])
        self.assertEqual(self.scene(**request).map_bounds, (0., 0., 4., 4.))
        with self.assertRaisesRegex(ValueError, "Target must be inside"):
            plan({**request, "targets": [{"id": 1, "x": 5, "y": 1}]})
        with self.assertRaisesRegex(ValueError, "fence_enabled"):
            plan({**request, "fence_enabled": "false"})

    def test_measured_fence_switch_matches_simulator_and_sdk_sweeps(self):
        config_path = Path(__file__).resolve().parents[2] / "sim/config/measured-fence.yaml"
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        config["robot"]["state"] = [1.2, .7, 0]
        vertices = config["custom"]["fence"]
        with TemporaryDirectory() as directory:
            path = Path(directory) / "world.yaml"
            for enabled in (True, False):
                with self.subTest(fence_enabled=enabled):
                    config["custom"]["fence_enabled"] = enabled
                    path.write_text(yaml.safe_dump(config), encoding="utf-8")
                    static = Scene(path, scenario="empty")
                    request = PlanningRequest.from_scene(static, method="stops")
                    self.assertEqual(request.to_dict()["fence"], vertices)
                    self.assertEqual((request.width, request.height), (3.936, 4.849))
                    sdk = self.scene(**request.to_dict())
                    restored = restore_scene(describe_scene(static))
                    for scene in (static, sdk, restored):
                        self.assertEqual(scene.config["custom"]["fence"], vertices)
                        self.assertEqual(scene.map_bounds, static.map_bounds)
                        self.assertEqual(scene.fence is not None, enabled)
                        drive, turn = scene.clone(), scene.clone()
                        drive.forward(.5)
                        turn.advance(rotation=math.pi / 2)
                        if enabled:
                            self.assertEqual(drive.failure, "Map boundary")
                            self.assertAlmostEqual(drive.pose[0], 1.3032098193904913)
                            self.assertEqual(turn.failure, "Map boundary")
                            self.assertAlmostEqual(turn.pose[2], 1.3482759132484485)
                        else:
                            for edge, expected in zip(scene.map_bounds, (-1.049, -3.139, 2.887, 1.71)):
                                self.assertAlmostEqual(edge, expected)
                            self.assertIsNone(drive.failure)
                            self.assertAlmostEqual(drive.pose[0], 1.7)
                            self.assertIsNone(turn.failure)
                            self.assertAlmostEqual(turn.pose[2], math.pi / 2)
                            drive.forward(2.)
                            self.assertEqual(drive.failure, "Map boundary")
                            self.assertAlmostEqual(drive.pose[0], 2.887 - scene.collector.xmax)

    def test_disabled_fence_is_shared_by_adjustment_and_escape(self):
        request = self.request(fence_enabled=False, pose=[-2., -2., 0.],
                               targets=[{"id": "a", "x": -1.22, "y": -2.}])
        adjusted = plan_pickup_adjustment(request, ["a"])
        self.assertTrue(adjusted.success, adjusted.reason)
        self.assertEqual(adjusted.expected_collected_ids, ["a"])
        self.assertLess(adjusted.final_pose[0], -1.)
        self.assertEqual(plan_escape(request).status, "already_clear")

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

    def test_auto_fence_limits_motion_and_can_reach_target_outside_circle(self):
        request = {"mode": "auto_fence", "pose": [-2., 3., 0.], "method": "stops", "use_ompl": False,
                   "targets": [{"id": "cup", "x": -.4, "y": 3.}]}
        result = plan(request)
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.expected_collected_ids, ["cup"])
        self.assertEqual(result.resolved_request["auto_fence_radius_m"], 1.4)
        scene, _, _ = _scene(PlanningRequest.from_dict(result.resolved_request))
        scene = restore_scene(json.loads(json.dumps(describe_scene(scene))))
        stops = [{"goal_index": stop.goal_index, "target_ids": [1]} for stop in result.pickup_stops]
        for dt in (.013, .04, .7):
            final = replay(scene, result.goals, dt, pickup_stops=stops, minimum_motion=True)
            self.assertIsNone(final.failure)
            self.assertFalse(final.targets)
            self.assertLessEqual(max(math.dist(scene.pose[:2], p) for p in final.path), 1.4 + 1e-9)
        smaller = plan({**request, "auto_fence_radius_m": .45})
        self.assertFalse(smaller.success)
        self.assertEqual(smaller.remaining_ids, ["cup"])
        self.assertFalse(smaller.goals)

    def test_auto_fence_center_stays_fixed_and_map_boundary_still_applies(self):
        start = [1., 1.5, 0.]
        fence = [(0., 0.), (2.5, 0.), (2.5, 3.), (0., 3.)]
        settings = {"pose": start, "fence": fence, "mode": "auto_fence"}
        scene = self.scene(**settings, auto_fence_radius_m=.6)
        scene.forward(.6)
        self.assertIsNone(scene.failure)
        scene.advance(rotation=math.pi / 2)
        self.assertIsNone(scene.failure)
        scene.forward(.2)
        self.assertEqual(scene.failure, "Motion circle boundary")
        self.assertAlmostEqual(math.dist(start[:2], scene.pose[:2]), .6)

        bounded = self.scene(**settings, auto_fence_radius_m=2.)
        bounded.forward(1.5)
        self.assertEqual(bounded.failure, "Map boundary")
        self.assertAlmostEqual(bounded.pose[0], 2.5 - bounded.collector.xmax)

    def test_auto_fence_options_are_validated(self):
        for radius in (0, -1., math.inf, math.nan, True):
            with self.subTest(radius=radius), self.assertRaisesRegex(ValueError, "auto_fence_radius_m"):
                self.scene(mode="auto_fence", auto_fence_radius_m=radius)
        with self.assertRaisesRegex(ValueError, "mode must be"):
            self.scene(mode="circle")
