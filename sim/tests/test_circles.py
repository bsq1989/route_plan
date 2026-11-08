"""Circle edges/corners, continuous swept motion and SDK lifecycle semantics."""

import math
from pathlib import Path
import random
import unittest

from route_plan_sim import Scene
from route_plan_sim.scene import Rect, normalize

LEGACY_ROBOT = Path(__file__).parent / "fixtures" / "robot.yaml"


class CircleContactTest(unittest.TestCase):
    rect = Rect(-.4, -.3, .4, .3)

    def test_corner_offset_is_round_not_an_inflated_square(self):
        self.assertFalse(self.rect.intersects_circle(.44, .34, .05))
        self.assertTrue(self.rect.intersects_circle(.43, .34, .05))
        self.assertEqual(self.rect.first_contact(.44, .34, .001, 0, .05), math.inf)

    def test_translation_corner_contact_between_clear_endpoints(self):
        t = self.rect.first_contact(2, .34, 4, 0, .05)
        self.assertAlmostEqual(t, (2 - .43) / 4)
        self.assertFalse(self.rect.intersects_circle(2, .34, .05))
        self.assertFalse(self.rect.intersects_circle(-2, .34, .05))

    def test_translation_tangency_and_gap(self):
        self.assertAlmostEqual(self.rect.first_contact(2, .35, 4, 0, .05), .4)
        self.assertEqual(self.rect.first_contact(2, .35001, 4, 0, .05), math.inf)

    def test_isolated_rotating_corner_tangency_both_directions(self):
        for rotation in (math.pi, -math.pi, math.tau):
            t = self.rect.first_contact(0, .55, 0, rotation, .05)
            self.assertAlmostEqual(t * abs(rotation), math.atan2(.4, .3), places=6)

    def test_asymmetric_collector_depends_on_rotation_direction(self):
        collector = Rect(.4, -.35, 1.1, .35)
        self.assertTrue(math.isfinite(collector.first_contact(0, .9, 0, math.pi, .05)))
        self.assertEqual(collector.first_contact(0, .9, 0, -math.pi, .05), math.inf)

    def test_zero_motion_includes_existing_circle_contact(self):
        self.assertEqual(self.rect.first_contact(.45, 0, 0, 0, .05), 0)
        self.assertEqual(self.rect.first_contact(.45001, 0, 0, 0, .05), math.inf)

    def test_random_contacts_agree_with_independent_sampled_distance(self):
        rng = random.Random(19)
        for _ in range(160):
            px, py = rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)
            radius = rng.uniform(.01, .15)
            distance, rotation = (rng.uniform(.1, 3), 0) if rng.random() < .5 else (0, rng.uniform(-6, 6))
            first = self.rect.first_contact(px, py, distance, rotation, radius)
            def distance_to_rectangle(t):
                c, s = math.cos(rotation * t), math.sin(rotation * t)
                x, y = c * px + s * py - distance * t, -s * px + c * py
                nearest_x, nearest_y = min(.4, max(-.4, x)), min(.3, max(-.3, y))
                return math.hypot(x - nearest_x, y - nearest_y)
            if math.isfinite(first):
                self.assertLessEqual(distance_to_rectangle(first), radius + 1e-8)
                if first > 1e-5:
                    self.assertGreater(distance_to_rectangle(first - 1e-5), radius - 1e-8)
            for index in range(1001):
                if distance_to_rectangle(index / 1000) <= radius:
                    self.assertLessEqual(first, index / 1000 + 1e-8)
                    break


class CircleSceneTest(unittest.TestCase):
    def empty(self):
        scene = Scene(robot_config=LEGACY_ROBOT, body_margin=0)
        scene.targets.clear()
        return scene

    def test_default_configuration_clone_and_snapshot_keep_diameter(self):
        scene = Scene()
        self.assertAlmostEqual(scene.target_diameter, .1)
        self.assertAlmostEqual(scene.target_radius, .05)
        self.assertEqual(scene.clone().snapshot(), scene.snapshot())
        self.assertEqual(scene.snapshot()["target_diameter"], .1)
        for value in (-.1, float("nan"), float("inf"), 20):
            with self.assertRaises(ValueError):
                Scene(target_diameter=value)

    def test_disk_outside_body_still_collides_with_margin(self):
        scene = self.empty()
        scene.set_dimensions(**(scene.dimensions | {"body_margin": .1, "collector_width": .2}))
        scene.add_target(5, 4.8)
        scene.forward(4)
        self.assertIsNotNone(scene.failure)
        self.assertAlmostEqual(scene.pose[0], 4.45)
        self.assertFalse(scene.safety_body.contains(*scene.local_point(5, 4.8)))
        self.assertTrue(scene.safety_body.intersects_circle(*scene.local_point(5, 4.8), .05))

    def test_collection_when_only_circle_edge_reaches_collector(self):
        scene = self.empty()
        scene.add_target(5, 4.5)
        scene.forward(.85)
        self.assertEqual(len(scene.collected), 1)
        self.assertFalse(scene.collector.contains(*scene.local_point(5, 4.5)))
        scene.forward(1)
        self.assertIsNone(scene.failure)

    def test_placement_requires_entire_circle_to_clear_safety_and_map(self):
        scene = self.empty()
        for x, y in ((3, 4.85), (.05, 2), (2, 8.96)):
            with self.assertRaises(ValueError):
                scene.add_target(x, y)
        scene.add_target(3, 4.85001)
        with self.assertRaises(ValueError):
            scene.set_dimensions(**(scene.dimensions | {"body_margin": .00002}))

    def test_overlapping_chain_collects_atomically_but_tangency_stays_separate(self):
        scene = Scene()
        scene.targets.clear()
        ids = scene.add_targets([(5, 4.5), (5.08, 4.5), (5.16, 4.5), (6, 4.5), (6.1, 4.5)])
        self.assertEqual(scene.target_groups, (ids[:3], (ids[3],), (ids[4],)))
        whole, framed = scene.clone(), scene.clone()
        whole.forward(1.461)
        for _ in range(1000):
            framed.forward(1.461 / 1000)
        for final in (whole, framed):
            self.assertIsNone(final.failure)
            self.assertEqual(set(final.collected), set(ids[:3]))
            self.assertEqual(set(final.targets), set(ids[3:]))
        whole.forward(6 - whole.pose[0] - whole.collector.xmax - whole.target_radius + .001)
        self.assertIn(ids[3], whole.collected)
        self.assertIn(ids[4], whole.targets)

    def test_batch_validation_precedes_any_collection(self):
        scene = Scene()
        scene.targets.clear()
        before = scene.snapshot()
        with self.assertRaises(ValueError):
            scene.add_targets([(3.44, 4.5), (3.39, 4.5)])
        self.assertEqual(scene.snapshot(), before)

    def test_group_outer_member_can_collide_before_tool_contact(self):
        for earlier in (.002, 0.):
            with self.subTest(earlier=earlier):
                scene = Scene()
                scene.targets.clear()
                first = (5, 4.7249)
                contact = scene.collector.first_contact(*scene.local_point(*first), 3, 0, scene.target_radius)
                other = (scene.pose[0] + 3 * contact + scene.safety_body.xmax + scene.target_radius - earlier,
                         first[1] + .02)
                bridge = ((first[0] + other[0]) / 2, (first[1] + other[1]) / 2)
                ids = scene.add_targets([first, bridge, other])
                self.assertEqual(scene.target_groups, (ids,))
                scene.forward(3)
                self.assertIsNotNone(scene.failure)
                self.assertFalse(scene.collected)
                self.assertEqual(set(scene.targets), set(ids))

    def test_group_cache_tracks_mutation_and_clone_is_independent(self):
        scene = self.empty()
        ids = scene.add_targets([(5, 4.5), (5.08, 4.5), (5.16, 4.5)])
        self.assertEqual(scene.target_groups, (ids,))
        clone = scene.clone()
        del clone.targets[ids[1]]
        self.assertEqual(clone.target_groups, ((ids[0],), (ids[2],)))
        self.assertEqual(scene.target_groups, (ids,))

    def test_front_margin_is_collision_even_at_collector_rear_edge(self):
        scene = Scene()
        scene.targets.clear()
        self.assertAlmostEqual(scene.collector_gap, scene.body_margin)
        self.assertAlmostEqual(scene.collector.xmin, scene.safety_body.xmax)
        x = scene.pose[0] + scene.safety_body.xmax + scene.target_radius
        with self.assertRaisesRegex(ValueError, "body"):
            scene.add_target(x, scene.pose[1])
        # A supplied bad state must also freeze at t=0 rather than be collected.
        scene.targets[999] = (x, scene.pose[1])
        before = scene.pose
        scene.forward(.1)
        self.assertIsNotNone(scene.failure)
        self.assertEqual(scene.pose, before)
        self.assertFalse(scene.collected)

    def test_circle_tie_prioritizes_body_collision_over_collection(self):
        scene = self.empty()
        scene.set_dimensions(**(scene.dimensions | {"collector_width": .6}))
        angle = .1
        scene.add_target(3 + .4 * math.cos(angle) - .35 * math.sin(angle),
                         4.5 + .4 * math.sin(angle) + .35 * math.cos(angle))
        scene.advance(rotation=.2)
        self.assertIsNotNone(scene.failure)
        self.assertFalse(scene.collected)
        self.assertAlmostEqual(scene.pose[2], angle)

    def test_random_placement_nonoverlap_and_reset_reproducibility(self):
        scene = Scene(scenario="random", seed=123)
        points = list(scene.targets.values())
        for i, (x, y) in enumerate(points):
            self.assertGreater(min(x, y, scene.width - x, scene.height - y), .05)
            self.assertFalse(scene.safety_body.intersects_circle(*scene.local_point(x, y), .05))
            for other in points[i + 1:]:
                self.assertGreaterEqual(math.dist((x, y), other), .1 - 1e-9)
        before = scene.snapshot()
        scene.forward(.2)
        scene.reset()
        self.assertEqual(scene.snapshot(), before)

    def test_whole_motion_and_small_steps_have_same_events(self):
        scene = self.empty()
        for point in ((4.8, 4.5), (5, 4.84), (5.8, 4.5)):
            scene.add_target(*point)
        for distance, rotation in ((3, 0), (0, math.pi), (0, -math.pi)):
            whole, framed = scene.clone(), scene.clone()
            whole.advance(distance=distance, rotation=rotation)
            for _ in range(1000):
                framed.advance(distance=distance / 1000, rotation=rotation / 1000)
            self.assertEqual(whole.targets, framed.targets)
            self.assertEqual(whole.collected, framed.collected)
            self.assertEqual(whole.failure, framed.failure)
            for a, b in zip(whole.pose[:2], framed.pose[:2]):
                self.assertAlmostEqual(a, b, places=7)
            self.assertAlmostEqual(normalize(whole.pose[2] - framed.pose[2]), 0, places=7)

    def test_rotation_tangency_is_independent_of_step_size(self):
        scene = self.empty()
        scene.set_dimensions(**(scene.dimensions | {"collector_width": .2}))
        scene.add_target(3, 5.05)
        for rotation in (math.pi, -math.pi):
            whole, framed = scene.clone(), scene.clone()
            whole.advance(rotation=rotation)
            for _ in range(1000):
                framed.advance(rotation=rotation / 1000)
            self.assertIsNotNone(whole.failure)
            self.assertEqual(whole.targets, framed.targets)
            self.assertEqual(whole.collected, framed.collected)
            self.assertEqual(whole.failure, framed.failure)
            self.assertAlmostEqual(normalize(whole.pose[2] - framed.pose[2]), 0, places=6)


if __name__ == "__main__":
    unittest.main()
