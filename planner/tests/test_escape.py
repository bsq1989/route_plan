import math
from pathlib import Path
import unittest

from route_plan_planner import plan, plan_escape
from route_plan_sim import PoseController, Scene

PREVIOUS_ROBOT = Path(__file__).resolve().parents[2] / "sim/tests/fixtures/robot-v0.2.4.yaml"


def clearance(rect, point, radius):
    x, y = point
    return math.hypot(max(rect.xmin - x, 0., x - rect.xmax),
                      max(rect.ymin - y, 0., y - rect.ymax)) - radius


class EscapeTest(unittest.TestCase):
    def request(self, points, pose=(3., 4.5, 0.)):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return dict(width=12, height=9, pose=pose, method="stops", use_ompl=False, timeout=2,
                    robot_config=PREVIOUS_ROBOT,
                    targets=[dict(id=f"object-{i}", x=x + c * u - s * v, y=y + s * u + c * v)
                             for i, (u, v) in enumerate(points)])

    def assert_escape_replays(self, request, response):
        self.assertEqual(response.status, "success", response.reason)
        self.assertEqual(response.goals[-1], response.final_pose)
        for dt in (.013, .7):
            scene = Scene(scenario="front", robot_config=request["robot_config"])
            scene.targets.clear()
            scene.pose = request["pose"]
            scene.path = [scene.pose[:2]]
            radius = scene.target_radius
            previous = {target["id"]: clearance(scene.body, scene.local_point(target["x"], target["y"]), radius)
                        for target in request["targets"] if target["id"] in response.initial_intrusion_ids}
            cleared = set()
            controller = PoseController(scene)
            for goal in response.goals:
                controller.set_goal(*goal)
                while controller.command is not None:
                    controller.step(dt)
                    self.assertIsNone(scene.failure)
                    for target in request["targets"]:
                        key = target["id"]
                        local = scene.local_point(target["x"], target["y"])
                        body_gap = clearance(scene.body, local, radius)
                        safety_gap = clearance(scene.safety_body, local, radius)
                        self.assertGreater(body_gap, 0.)
                        if key in cleared or key not in response.initial_intrusion_ids:
                            self.assertGreater(safety_gap, 0.)
                        if key in previous:
                            self.assertGreaterEqual(body_gap + 1e-8, previous[key])
                        if key in response.initial_intrusion_ids:
                            previous[key] = body_gap
                        if safety_gap > 0.:
                            cleared.add(key)
            self.assertEqual(cleared, {target["id"] for target in request["targets"]})
            for actual, expected in zip(scene.pose, response.final_pose):
                self.assertAlmostEqual(actual, expected)

    def test_rear_and_side_can_exit_and_then_replan_with_all_targets(self):
        for points, pose in (([(-.39, 0)], (3., 4.5, 0.)),
                             ([(0, .44)], (3., 4.5, .7)),
                             ([(-.39, 0), (0, .44)], (3., 4.5, 0.))):
            with self.subTest(points=points, pose=pose):
                request = self.request(points, pose)
                with self.assertRaisesRegex(ValueError, "inside the robot body"):
                    plan(dict(request, method="length"))
                response = plan_escape(request)
                self.assert_escape_replays(request, response)
                self.assertEqual(response.initial_intrusion_ids, tuple(t["id"] for t in request["targets"]))
                resumed = plan(dict(request, pose=response.final_pose))
                self.assertTrue(resumed.success, resumed.reason)
                self.assertEqual(set(resumed.expected_collected_ids), set(response.initial_intrusion_ids))

    def test_corner_intrusion_can_exit_by_turning_away(self):
        request = self.request([(.37, .4)])
        response = plan_escape(request)
        self.assert_escape_replays(request, response)
        self.assertEqual(response.final_pose[:2], request["pose"][:2])
        self.assertGreater(response.final_pose[2], 0.)

    def test_turn_then_forward_escape_replays_both_actions(self):
        request = self.request([(.26, .355)])
        response = plan_escape(request)
        self.assert_escape_replays(request, response)
        self.assertEqual(len(response.goals), 2)
        self.assertEqual(response.goals[0][:2], request["pose"][:2])
        self.assertNotEqual(response.goals[1][:2], request["pose"][:2])

    def test_cleared_intrusion_keeps_body_clearance_during_following_move(self):
        request = self.request([(.297, -.43), (-.12, .38)], (3., 4.5, .7))
        response = plan_escape(request)
        self.assert_escape_replays(request, response)
        self.assertEqual(len(response.goals), 2)

    def test_both_a_front_obstacle_and_a_wall_can_block_forward_escape(self):
        for request in (self.request([(0, .44), (.53, 0)]),
                        self.request([(0, .44)], (11.35, 4.5, 0.))):
            response = plan_escape(request)
            self.assertEqual(response.status, "blocked")
            self.assertEqual(response.goals, ())

    def test_unsafe_start_or_no_separating_move_returns_no_goals(self):
        for points, status in (([(.30, 0)], "body_contact"), ([(.39, 0)], "blocked")):
            response = plan_escape(self.request(points))
            self.assertEqual(response.status, status)
            self.assertFalse(response.success)
            self.assertEqual(response.goals, ())
            self.assertIsNone(response.final_pose)

    def test_already_clear_and_invalid_observation(self):
        request = self.request([(.8, 0)])
        response = plan_escape(request)
        self.assertEqual(response.status, "already_clear")
        self.assertEqual(response.final_pose, request["pose"])
        self.assertEqual(response.goals, ())
        with self.assertRaisesRegex(ValueError, "inside the map"):
            plan_escape(self.request([(-3, 0)]))


if __name__ == "__main__":
    unittest.main()
