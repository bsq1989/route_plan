import math
from copy import deepcopy
from pathlib import Path
import unittest

from route_plan_sim import Scene

LEGACY_ROBOT = Path(__file__).parent / "fixtures" / "robot.yaml"


class SceneRulesTest(unittest.TestCase):
    def empty(self):
        scene = Scene(body_margin=0, robot_config=LEGACY_ROBOT)
        scene.targets.clear()
        return scene

    def test_forward_collects_multiple_points_before_body_arrives(self):
        scene = Scene(robot_config=LEGACY_ROBOT)
        scene.forward(1.5)
        self.assertIsNone(scene.failure)
        self.assertEqual(len(scene.collected), 3)
        self.assertEqual(len(scene.targets), 7)
        self.assertAlmostEqual(scene.pose[0], 4.5)

    def test_full_translation_cannot_skip_a_point(self):
        scene = self.empty()
        scene.add_target(5, 4.5)
        scene.forward(4)
        self.assertIsNone(scene.failure)
        self.assertEqual(len(scene.collected), 1)

    def test_rotation_collision_between_clear_endpoints(self):
        scene = self.empty()
        scene.add_target(3, 4.99)
        scene.advance(rotation=math.pi)
        self.assertIn("Body hit", scene.failure)
        self.assertLess(scene.pose[2], math.pi / 2)
        self.assertFalse(scene.collected)
        pose = scene.pose
        scene.forward(1)
        self.assertEqual(scene.pose, pose)

    def test_rotation_corner_tangency_is_collision(self):
        scene = self.empty()
        scene.add_target(3, 5)  # body corner radius = hypot(0.4, 0.3) = 0.5
        scene.advance(rotation=math.pi)
        self.assertIsNotNone(scene.failure)
        self.assertAlmostEqual(scene.pose[2], math.atan2(.4, .3))

    def test_collector_is_active_while_turning(self):
        scene = self.empty()
        scene.add_target(3.7, 5.2)
        scene.turn_to(90)
        self.assertIsNone(scene.failure)
        self.assertEqual(len(scene.collected), 1)

    def test_reverse_combined_motion_and_nan_are_rejected(self):
        scene = self.empty()
        for action in (lambda: scene.forward(-1),
                       lambda: scene.advance(distance=1, rotation=.5),
                       lambda: scene.turn_to(float("nan")),
                       lambda: scene.forward(float("inf"))):
            with self.assertRaises(ValueError):
                action()

    def test_target_on_body_edge_is_rejected(self):
        scene = self.empty()
        with self.assertRaises(ValueError):
            scene.add_target(3, 4.8)

    def test_collector_edge_counts_as_collection(self):
        scene = self.empty()
        scene.add_target(5, 4.85)
        scene.forward(2)
        self.assertIsNone(scene.failure)
        self.assertEqual(len(scene.collected), 1)

    def test_simultaneous_body_and_collector_contact_fails(self):
        scene = self.empty()
        # A mathematical point at the shared rear collector/body corner enters
        # both regions at exactly the same angle. Body contact wins that tie.
        a = .2
        x = .4 * math.cos(a) - .3 * math.sin(a)
        y = .4 * math.sin(a) + .3 * math.cos(a)
        scene.add_target(3 + x, 4.5 + y)
        scene.advance(rotation=.4)
        self.assertIsNotNone(scene.failure)
        self.assertFalse(scene.collected)

    def test_full_turn_and_wrapped_absolute_angle(self):
        scene = self.empty()
        scene.advance(rotation=math.tau)
        self.assertIsNone(scene.failure)
        self.assertAlmostEqual(scene.pose[2], 0)
        scene.turn_to(450)
        self.assertAlmostEqual(scene.pose[2], math.pi / 2)
        self.assertEqual(scene.pose[:2], scene.initial_pose[:2])

    def test_forward_stops_when_collector_reaches_map_edge(self):
        scene = self.empty()
        scene.forward(20)
        self.assertEqual(scene.failure, "Map boundary")
        self.assertAlmostEqual(scene.pose[0], 10.9)

    def test_rotation_checks_swept_collector_at_wall(self):
        scene = self.empty()
        scene.pose = (10.88, 4.5, 0)
        # Start and end fit, but the lower collector corner crosses x=12
        # between 0 and 90 degrees.
        scene.turn_to(90)
        self.assertEqual(scene.failure, "Map boundary")
        self.assertLess(scene.pose[2], math.pi / 2)

    def test_collections_after_collision_are_not_applied(self):
        scene = self.empty()
        scene.add_target(3, 4.99)
        scene.add_target(2, 4.5)
        scene.advance(rotation=math.pi)
        self.assertEqual(len(scene.targets), 2)
        self.assertFalse(scene.collected)

    def test_random_is_reproducible_and_reset_restores_targets(self):
        scene = Scene(scenario="random", seed=7, robot_config=LEGACY_ROBOT)
        before = scene.snapshot()
        self.assertEqual(len(scene.targets), 40)
        scene.turn_to(60)
        scene.forward(.5)
        scene.reset()
        self.assertEqual(scene.snapshot(), before)
        self.assertNotEqual(scene.targets, Scene(scenario="random", seed=8, robot_config=LEGACY_ROBOT).targets)

    def test_resize_preserves_pose_and_collects_newly_covered_points(self):
        scene = self.empty()
        scene.turn_to(90)
        scene.add_target(3.5, 5.4)
        before_pose = scene.pose
        scene.set_dimensions(body_length=1.2, body_width=.7, collector_width=1.2, collector_depth=.8)
        self.assertEqual(scene.pose, before_pose)
        self.assertEqual(len(scene.collected), 1)
        self.assertEqual(scene.collector.xmin, .6)
        self.assertEqual(scene.config["robot"]["shape"]["length"], 1.2)
        self.assertEqual(scene.snapshot()["dimensions"]["collector_width"], 1.2)

    def test_invalid_resize_does_not_mutate_episode_or_config(self):
        scene = self.empty()
        scene.add_target(3, 5)
        before = deepcopy(scene.snapshot())
        config = deepcopy(scene.config)
        invalid = [dict(body_length=.8, body_width=1.1, collector_width=.7, collector_depth=.7),
                   dict(body_length=8, body_width=.6, collector_width=.7, collector_depth=.7),
                   dict(body_length=.8, body_width=.6, collector_width=.7, collector_depth=15)]
        for value in (0, -1, float("nan"), float("inf")):
            invalid.append(scene.dimensions | {"body_length": value})
        for dimensions in invalid:
            with self.subTest(dimensions=dimensions):
                with self.assertRaises(ValueError):
                    scene.set_dimensions(**dimensions)
                self.assertEqual(scene.snapshot(), before)
                self.assertEqual(scene.config, config)

    def test_narrow_collector_changes_forward_collision(self):
        scene = self.empty()
        scene.add_target(5, 4.75)
        scene.set_dimensions(**(scene.dimensions | {"collector_width": .2}))
        scene.forward(2)
        self.assertIsNotNone(scene.failure)
        self.assertFalse(scene.collected)
        self.assertAlmostEqual(scene.pose[0], 4.6)

    def test_reset_retains_applied_dimensions(self):
        scene = Scene(robot_config=LEGACY_ROBOT)
        scene.set_dimensions(body_length=1.1, body_width=.5, collector_width=.9, collector_depth=.4)
        dimensions = scene.dimensions
        scene.forward(.3)
        scene.reset("random")
        self.assertEqual(scene.dimensions, dimensions)
        self.assertEqual(scene.pose, scene.initial_pose)
        self.assertEqual(len(scene.targets), 40)

    def test_invalid_preset_after_resize_leaves_scene_intact(self):
        scene = self.empty()
        scene.set_dimensions(body_length=4.2, body_width=1, collector_width=.7, collector_depth=.2)
        before = scene.snapshot()
        with self.assertRaises(ValueError):
            scene.reset("front")
        self.assertEqual(scene.snapshot(), before)
        scene.reset("random")
        self.assertEqual(len(scene.targets), 40)

    def test_safety_margin_boundary_contact_fails_without_body_contact(self):
        scene = self.empty()
        scene.set_dimensions(**(scene.dimensions | {"body_margin": .1}))
        scene.add_target(5, 4.9)  # On the top safety edge, outside physical body/collector.
        scene.forward(2)
        self.assertIsNotNone(scene.failure)
        self.assertAlmostEqual(scene.pose[0], 4.5)
        self.assertFalse(scene.body.contains(*scene.local_point(5, 4.9)))
        self.assertTrue(scene.safety_body.contains(*scene.local_point(5, 4.9)))

    def test_point_outside_safety_margin_is_not_a_collision(self):
        scene = self.empty()
        scene.set_dimensions(**(scene.dimensions | {"body_margin": .1}))
        scene.add_target(5, 4.90001)
        scene.forward(2)
        self.assertIsNone(scene.failure)
        self.assertEqual(len(scene.targets), 1)

    def test_rotating_expanded_corner_contact_fails(self):
        scene = self.empty()
        scene.set_dimensions(**(scene.dimensions | {"body_margin": .1}))
        scene.add_target(3, 4.5 + math.hypot(.5, .4))
        scene.advance(rotation=math.pi)
        self.assertIsNotNone(scene.failure)
        self.assertAlmostEqual(scene.pose[2], math.atan2(.5, .4))

    def test_margin_resize_and_placement_respect_inclusive_boundary(self):
        scene = self.empty()
        scene.set_dimensions(**(scene.dimensions | {"body_margin": .1}))
        with self.assertRaises(ValueError):
            scene.add_target(3, 4.9)
        scene.add_target(3, 4.95)
        before = deepcopy(scene.snapshot())
        for margin in (.2, -.1, float("nan")):
            with self.assertRaises(ValueError):
                scene.set_dimensions(**(scene.dimensions | {"body_margin": margin}))
            self.assertEqual(scene.snapshot(), before)
        scene.reset("random")
        self.assertEqual(scene.body_margin, .1)


if __name__ == "__main__":
    unittest.main()
