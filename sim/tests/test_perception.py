from importlib.util import find_spec
import math
import unittest

from route_plan_sim import PoseController, Scene


class PerceptionSafetyTest(unittest.TestCase):
    def test_fixed_occluder_stops_forward_motion_at_first_contact(self):
        scene = Scene(scenario="empty", obstacles=[(4.3, 4.5, .3)])
        motion = PoseController(scene)
        motion.set_goal(5, 4.5, 0)
        motion.step(10)
        self.assertEqual(scene.failure, "Hit obstacle #1")
        self.assertAlmostEqual(scene.pose[0], 4.3 - .3 - scene.collector.xmax)
        self.assertEqual(scene.clone().obstacles, scene.obstacles)

    def test_unobserved_targets_still_stop_the_car(self):
        from route_plan_sim.cameras import Camera

        scene = Scene(scenario="empty")
        scene.add_target(4.2, 4.8)
        camera = Camera("local", (.27, .04, 0), math.radians(10), 2.8)
        frame, _ = camera.capture(scene, 0)
        self.assertEqual(frame["detections"], [])
        scene.forward(2)
        self.assertIn("Body hit target", scene.failure)


@unittest.skipUnless(find_spec("active_perception_sdk"), "requires the local active perception SDK")
class PerceptionDemoTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from route_plan_sim.perception_demo import load_demo, run_case

        config = load_demo()
        cls.results = {name: run_case(config, name, 42)[1] for name in config["cases"]}

    def test_far_bias_preserves_bearing_and_local_range_limit(self):
        from route_plan_sim.cameras import to_local

        result = self.results["range-bias"]
        first, turned = result["observations"][:2]
        cue, = first["bottom"]["detections"]
        target, = result["truth"]["targets"].values()
        exact_xy = to_local(first["bottom"]["camera_pose_map"], target)
        self.assertAlmostEqual(cue["bearing_rad"], math.atan2(exact_xy[1], exact_xy[0]))
        self.assertAlmostEqual(math.atan2(cue["xy_m"][1], cue["xy_m"][0]), cue["bearing_rad"])
        self.assertGreater(math.hypot(*cue["xy_m"]), math.hypot(*exact_xy))
        self.assertEqual(turned["stage"], "sdk_view")
        self.assertEqual(turned["local"]["detections"], [])
        self.assertEqual(set(turned["truth_visibility"]["local"].values()), {"out_of_range"})
        self.assertNotEqual(result["actions"][0]["start_pose"], result["actions"][0]["end_pose"])

    def test_occlusion_resolves_after_real_translation(self):
        result = self.results["occlusion"]
        first = result["observations"][0]
        self.assertEqual(set(first["truth_visibility"]["local"].values()), {"occluded"})
        self.assertEqual(first["local"]["detections"], [])
        self.assertEqual(first["bottom"]["detections"], [])
        self.assertGreater(math.dist(result["trace"][0]["pose"][:2], result["trace"][-1]["pose"][:2]), 1)
        self.assertIsNone(result["final"]["failure"])
        self.assertEqual(len(result["handoff"]["targets_xy_m"]), 1)

    def test_one_wide_cue_can_yield_multiple_local_targets(self):
        result = self.results["shared-bearing"]
        self.assertEqual(len(result["observations"][0]["bottom"]["detections"]), 1)
        self.assertEqual(len(result["view_requests"][0]["suggestions"]), 1)
        self.assertEqual(len(result["handoff"]["targets_xy_m"]), 2)

    def test_handoff_is_map_coordinates_from_local_frame(self):
        for result in self.results.values():
            with self.subTest(case=result["case"]):
                handoff = result["handoff"]
                self.assertEqual(handoff["frame_id"], "map")
                local = result["observations"][handoff["observation_index"]]["local"]
                self.assertEqual(handoff["timestamp_s"], local["timestamp_s"])
                self.assertEqual(len(handoff["targets_xy_m"]), len(local["detections"]))
                self.assertEqual(len(result["truth"]["targets"]), len(result["final"]["remaining"]))
                for xy in handoff["targets_xy_m"]:
                    self.assertLess(min(math.dist(xy, target) for target in result["truth"]["targets"].values()), 1e-8)
