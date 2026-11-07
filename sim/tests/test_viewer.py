"""Offscreen integration checks for the IR-SIM adapter and manual commands."""

import os
os.environ["MPLBACKEND"] = "Agg"

import unittest
import json
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from route_plan_sim import Scene
from route_plan_sim.app import Viewer


class ViewerTest(unittest.TestCase):
    def setUp(self):
        self.viewer = Viewer(Scene(robot_config=Path(__file__).parent / "fixtures" / "robot.yaml"), display=False)

    def tearDown(self):
        self.viewer.env.end()

    def finish_action(self):
        for _ in range(200):
            self.viewer.tick()
            if self.viewer.command is None:
                return
        self.fail("Manual action did not finish")

    def test_forward_updates_irsim_pose_and_collects_targets(self):
        self.viewer.start("move", 1.5)
        self.finish_action()
        self.assertEqual(len(self.viewer.scene.collected), 3)
        np.testing.assert_allclose(self.viewer.env.robot.state.flatten(), self.viewer.scene.pose)
        self.assertEqual(len(self.viewer.targets.get_offsets()), 7)

    def test_absolute_heading_and_pause(self):
        self.viewer.heading_box.set_val("450")
        self.viewer.send_goal()
        self.viewer.toggle_pause()
        pose = self.viewer.scene.pose
        self.viewer.tick()
        self.assertEqual(self.viewer.scene.pose, pose)
        self.viewer.toggle_pause()
        self.finish_action()
        self.assertAlmostEqual(np.degrees(self.viewer.scene.pose[2]), 90)
        self.assertEqual(self.viewer.scene.pose[:2], pose[:2])

    def test_negative_distance_does_not_start_action(self):
        self.viewer.start("move", -1)
        self.assertIsNone(self.viewer.command)
        self.assertIn("不能倒车", self.viewer.message.get_text())

    def test_reset_clears_motion_and_switches_scenario(self):
        self.viewer.start("move", .25)
        self.viewer.tick()
        self.viewer.reset("surround")
        self.assertIsNone(self.viewer.command)
        self.assertEqual(self.viewer.scene.pose, self.viewer.scene.initial_pose)
        self.assertEqual(len(self.viewer.scene.targets), 32)
        self.assertFalse(self.viewer.scene.collected)

    def test_status_does_not_overlap_buttons(self):
        self.viewer.fig.canvas.draw()
        renderer = self.viewer.fig.canvas.get_renderer()
        status = self.viewer.status.get_window_extent(renderer)
        self.assertFalse(status.overlaps(self.viewer.pause_button.ax.get_window_extent(renderer)))
        save_button = self.viewer.widgets[-1]
        self.assertFalse(status.overlaps(save_button.ax.get_window_extent(renderer)))
        self.assertFalse(status.overlaps(self.viewer.ax.xaxis.label.get_window_extent(renderer)))

    def test_apply_resizes_irsim_and_collector_and_stops_action(self):
        self.viewer.scene.turn_to(45)
        self.viewer.sync()
        self.viewer.start("move", 1)
        dimensions = dict(body_length=1.2, body_width=.8, collector_width=1.3, collector_depth=.9)
        for key, value in dimensions.items():
            self.viewer.dimension_boxes[key].set_val(str(value))
        # Editing the four fields is atomic: only the apply button commits.
        self.assertEqual(self.viewer.scene.dimensions["body_length"], .8)
        before_pose = self.viewer.scene.pose
        self.viewer.apply_dimensions()
        self.assertIsNone(self.viewer.command)
        self.assertEqual(self.viewer.scene.pose, before_pose)
        self.viewer.env.render(.001)
        self.assertAlmostEqual(self.viewer.env.robot.length, 1.2)
        self.assertAlmostEqual(self.viewer.env.robot.width, .8)
        np.testing.assert_allclose(self.viewer.env.robot.vertices.T,
                                   self.viewer.scene.vertices(self.viewer.scene.body), atol=1e-9)
        np.testing.assert_allclose(self.viewer.collector.get_xy()[:4],
                                   self.viewer.scene.vertices(self.viewer.scene.collector), atol=1e-9)
        # Verify the actual body artist as well as the geometry object.
        patch = self.viewer.env.robot.plot_patch_list[0]
        vertices = self.viewer.ax.transData.inverted().transform(
            patch.get_transform().transform(patch.get_path().vertices))
        expected = np.array(self.viewer.scene.vertices(self.viewer.scene.body))
        np.testing.assert_allclose(vertices[:4], expected, atol=1e-9)
        with TemporaryDirectory() as directory:
            path = self.viewer.save(Path(directory) / "resized.png")
            saved = json.loads(path.with_suffix(".json").read_text())
            for key, value in dimensions.items():
                self.assertAlmostEqual(saved["dimensions"][key], value)
            self.assertEqual(saved["config"]["robot"]["shape"]["width"], .8)

    def test_invalid_dimension_does_not_change_geometry(self):
        self.viewer.dimension_boxes["body_width"].set_val("-1")
        before = self.viewer.scene.snapshot()
        self.viewer.apply_dimensions()
        self.assertEqual(self.viewer.scene.snapshot(), before)
        self.assertIn("大于 0", self.viewer.message.get_text())
        self.assertAlmostEqual(self.viewer.env.robot.width, .6)

    def test_map_pose_fields_and_goal_outline(self):
        self.viewer.scene.targets.clear()
        for key, value in dict(x=4, y=5, yaw=37).items():
            self.viewer.pose_boxes[key].set_val(str(value))
        self.assertIsNone(self.viewer.command)
        self.viewer.send_goal()
        self.finish_action()
        np.testing.assert_allclose(self.viewer.scene.pose, (4, 5, np.radians(37)), atol=1e-8)
        self.assertTrue(self.viewer.goal_outline.get_visible())
        self.assertEqual(self.viewer.state_badge.get_text(), "●  已到达")

    def test_rear_pose_is_rejected_and_margin_is_drawn(self):
        self.viewer.pose_boxes["x"].set_val("2")
        self.viewer.send_goal()
        self.assertIsNone(self.viewer.command)
        self.assertIn("车后方", self.viewer.message.get_text())
        self.viewer.dimension_boxes["body_margin"].set_val("0.2")
        self.viewer.apply_dimensions()
        np.testing.assert_allclose(self.viewer.safety_outline.get_xy()[:4],
                                   self.viewer.scene.vertices(self.viewer.scene.safety_body))

    def test_default_profile_matches_real_robot_and_display(self):
        viewer = Viewer(Scene(), display=False)
        try:
            self.assertEqual(viewer.dimension_boxes["body_length"].text, "0.5")
            self.assertEqual(viewer.dimension_boxes["collector_width"].text, "0.24")
            self.assertEqual(viewer.dimension_boxes["collector_depth"].text, "0.35")
            self.assertAlmostEqual(viewer.env.robot.geometry.area, .5 * .6)
            self.assertAlmostEqual(viewer.scene.collector.xmin, .25)
            np.testing.assert_allclose(viewer.collector.get_xy()[:4],
                                       viewer.scene.vertices(viewer.scene.collector))
        finally:
            viewer.env.end()


if __name__ == "__main__":
    unittest.main()
