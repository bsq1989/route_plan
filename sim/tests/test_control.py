import math
from pathlib import Path
import unittest

from route_plan_sim import PoseController, Scene


class PoseControllerTest(unittest.TestCase):
    def setUp(self):
        self.scene = Scene(robot_config=Path(__file__).parent / "fixtures" / "robot.yaml")
        self.scene.targets.clear()
        self.controller = PoseController(self.scene)

    def finish(self):
        for _ in range(1000):
            self.controller.step()
            if self.controller.command is None:
                return
        self.fail("Pose command did not finish")

    def test_rotate_then_move_then_final_yaw(self):
        self.controller.set_goal(4, 5.5, -.3)
        self.controller.step(.1)
        self.assertEqual(self.scene.pose[:2], (3, 4.5))
        self.assertGreater(self.scene.pose[2], 0)
        self.finish()
        self.assertIsNone(self.scene.failure)
        for actual, expected in zip(self.scene.pose, (4, 5.5, -.3)):
            self.assertAlmostEqual(actual, expected)

    def test_arbitrary_yaw_can_be_updated_during_rotation(self):
        self.controller.set_goal(3, 4.5, 1.1)
        self.controller.step(.1)
        self.controller.set_goal(3, 4.5, -.37)
        self.finish()
        self.assertAlmostEqual(self.scene.pose[2], -.37)
        self.assertEqual(self.scene.pose[:2], (3, 4.5))

    def test_rear_check_uses_current_heading_not_map_x(self):
        self.scene.turn_to(90)
        self.controller.set_goal(3, 5, 1)
        previous_goal, previous_command = self.controller.goal, self.controller.command
        with self.assertRaisesRegex(ValueError, "behind"):
            self.controller.set_goal(4, 4, 0)
        self.assertEqual(self.controller.goal, previous_goal)
        self.assertEqual(self.controller.command, previous_command)
        self.assertEqual(self.scene.pose[:2], (3, 4.5))

    def test_sideways_goal_is_allowed_but_never_moves_backward(self):
        self.controller.set_goal(3, 5.5, math.pi)
        previous = self.scene.pose
        while self.controller.command:
            self.controller.step()
            x, y, _ = self.scene.pose
            dx, dy = x - previous[0], y - previous[1]
            self.assertGreaterEqual(dx * math.cos(previous[2]) + dy * math.sin(previous[2]), -1e-8)
            previous = self.scene.pose
        self.assertAlmostEqual(self.scene.pose[1], 5.5)

    def test_pause_and_stop(self):
        self.controller.set_goal(4, 4.5, 0)
        self.controller.paused = True
        self.controller.step(1)
        self.assertEqual(self.scene.pose[:2], (3, 4.5))
        self.controller.paused = False
        self.controller.step(.1)
        self.assertGreater(self.scene.pose[0], 3)
        self.controller.stop()
        pose = self.scene.pose
        self.controller.step(10)
        self.assertEqual(self.scene.pose, pose)
        self.assertIsNone(self.controller.goal)

    def test_collision_aborts_remainder_of_pose_command(self):
        self.scene.add_target(4, 4.91)  # Circle clears collector, hits safety corner.
        self.controller.set_goal(5, 4.5, 1)
        self.finish()
        self.assertIsNotNone(self.scene.failure)
        self.assertIsNone(self.controller.command)
        self.assertAlmostEqual(self.scene.pose[0], 3.5 - math.sqrt(.05**2 - .01**2))
        self.assertAlmostEqual(self.scene.pose[2], 0)

    def test_goal_replacement_starts_from_current_pose(self):
        self.controller.set_goal(5, 4.5, 0)
        self.controller.step(.5)
        self.controller.set_goal(4, 5, .6)
        self.finish()
        for actual, expected in zip(self.scene.pose, (4, 5, .6)):
            self.assertAlmostEqual(actual, expected)

    def test_invalid_commands_leave_existing_goal_intact(self):
        self.controller.set_goal(5, 4.5, 0)
        for goal in ((2, 4.5, 0), (13, 5, 0), (4, float("nan"), 0), (4, 5, float("inf"))):
            previous_goal, previous_command = self.controller.goal, self.controller.command
            with self.assertRaises(ValueError):
                self.controller.set_goal(*goal)
            self.assertEqual(self.controller.goal, previous_goal)
            self.assertEqual(self.controller.command, previous_command)


if __name__ == "__main__":
    unittest.main()
