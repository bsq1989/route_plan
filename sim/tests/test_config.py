from copy import deepcopy
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

import yaml

from route_plan_sim import Scene
from route_plan_sim.scene import DEFAULT_CONFIG

RECTANGULAR_ROBOT = Path(__file__).parent / "fixtures" / "robot-rectangular.yaml"


class RobotConfigTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.folder = Path(self.directory.name)
        self.profile = yaml.safe_load(RECTANGULAR_ROBOT.read_text())

    def write_profile(self, profile=None):
        path = self.folder / "robot.yaml"
        path.write_text(yaml.safe_dump(self.profile if profile is None else profile), encoding="utf-8")
        return path

    def test_real_robot_is_the_default_for_each_new_episode(self):
        scene = Scene()
        for key, expected in dict(body_length=.702, body_width=.610, collector_width=.466427,
                                  collector_depth=.247522, body_margin=.050).items():
            self.assertAlmostEqual(scene.dimensions[key], expected)
        self.assertAlmostEqual(scene.config["custom"]["body_height"], .169)
        self.assertEqual(scene.body_pose_reference, "base_link")
        self.assertAlmostEqual(scene.body.xmax, .341)
        self.assertAlmostEqual(scene.body.xmin, -.361)
        self.assertAlmostEqual(scene.collector_gap, .059531)
        self.assertEqual(len(scene.collector.corners), 241)
        self.assertTrue(scene.collector.contains(.56, .14))
        self.assertFalse(scene.collector.contains(.43, 0))
        self.assertAlmostEqual(scene.safety_body.xmax - scene.safety_body.xmin, .802)
        self.assertAlmostEqual(scene.safety_body.ymax - scene.safety_body.ymin, .710)
        scene.set_dimensions(**(scene.dimensions | {"body_length": .7}))
        self.assertAlmostEqual(Scene().dimensions["body_length"], .702)

    def test_default_does_not_depend_on_working_directory(self):
        code = "from route_plan_sim import Scene; s=Scene(); assert s.dimensions['body_length']==0.702; print('ok')"
        result = subprocess.run([sys.executable, "-c", code], cwd=self.folder,
                                check=True, capture_output=True, text=True)
        self.assertEqual(result.stdout.strip(), "ok")

    def test_file_edits_are_read_by_next_initialization(self):
        path = self.write_profile()
        scene = Scene(robot_config=path)
        self.profile["body"]["length"] = .65
        self.profile["motion"]["forward_speed"] = .9
        self.write_profile()
        next_scene = Scene(robot_config=path)
        self.assertAlmostEqual(scene.dimensions["body_length"], .702)
        self.assertAlmostEqual(next_scene.dimensions["body_length"], .65)
        self.assertAlmostEqual(next_scene.config["custom"]["forward_speed"], .9)

    def test_relative_scene_reference_and_explicit_override(self):
        path = self.write_profile()
        world = yaml.safe_load(Path(str(DEFAULT_CONFIG)).read_text())
        world["custom"]["robot_config"] = "robot.yaml"
        world_path = self.folder / "world.yaml"
        world_path.write_text(yaml.safe_dump(world))
        self.assertEqual(Scene(config=world_path).robot_config_path, path.resolve())
        world["custom"]["robot_config"] = "missing.yaml"
        world_path.write_text(yaml.safe_dump(world))
        self.assertEqual(Scene(config=world_path, robot_config=path).robot_config_path, path.resolve())
        with self.assertRaises(FileNotFoundError):
            Scene(config=world_path)

    def test_explicit_gap_and_lateral_offset_survive_resizing(self):
        self.profile["collector"].update(gap=.05, lateral_offset=.03)
        scene = Scene(robot_config=self.write_profile())
        scene.targets.clear()
        self.assertAlmostEqual(scene.collector.xmin, .401)
        self.assertAlmostEqual(scene.collector.ymin, -.065)
        scene.set_dimensions(**(scene.dimensions | {"body_length": .8}))
        self.assertAlmostEqual(scene.collector.xmin, .45)
        self.assertAlmostEqual(scene.collector.ymax, .125)
        scene.set_dimensions(**(scene.dimensions | {"body_margin": .15}))
        self.assertAlmostEqual(scene.collector_gap, .05)
        self.assertAlmostEqual(scene.collector.xmin, .45)
        self.assertAlmostEqual(scene.safety_body.xmax, .55)

    def test_omitted_gap_follows_body_margin(self):
        del self.profile["collector"]["gap"]
        scene = Scene(robot_config=self.write_profile(), body_margin=.12)
        scene.targets.clear()
        self.assertAlmostEqual(scene.collector_gap, .12)
        scene.set_dimensions(**(scene.dimensions | {"body_margin": .15}))
        self.assertAlmostEqual(scene.collector_gap, .15)

    def test_zero_gap_keeps_collision_priority_where_collector_overlaps_margin(self):
        self.profile["collector"]["gap"] = 0
        scene = Scene(robot_config=self.write_profile())
        scene.targets.clear()
        self.assertAlmostEqual(scene.collector.xmin, scene.body.xmax)
        x, y, _ = scene.pose
        self.assertTrue(scene.collector.contains(.37, 0))
        with self.assertRaisesRegex(ValueError, "body"):
            scene.add_target(x + .37, y)
        scene.add_target(x + .46, y)
        self.assertEqual(len(scene.collected), 1)

    def test_invalid_profile_never_falls_back_to_demo_sizes(self):
        profiles = []
        for section, key, value in (("body", "length", 0), ("body", "height", 0), ("safety", "body_margin", -.1),
                                     ("collector", "depth", float("nan")), ("collector", "gap", -.01),
                                     ("motion", "angular_speed", float("inf")),
                                     ("body", "pose_reference", "rear_axle")):
            profile = deepcopy(self.profile)
            profile[section][key] = value
            profiles.append(profile)
        profiles.append(self.profile | {"units": "cm"})
        for profile in profiles:
            with self.subTest(profile=profile):
                with self.assertRaises(ValueError):
                    Scene(robot_config=self.write_profile(profile))
        with self.assertRaises(FileNotFoundError):
            Scene(robot_config=self.folder / "absent.yaml")

    def test_real_narrow_collector_does_not_remove_shoulder_points(self):
        scene = Scene(robot_config=RECTANGULAR_ROBOT)
        scene.targets.clear()
        scene.add_target(4.8, 4.8)
        scene.forward(1.5)
        self.assertIsNotNone(scene.failure)
        self.assertAlmostEqual(scene.pose[0], 4.349)  # 5 cm radius contacts sooner.
        self.assertIn((4.8, 4.8), scene.targets.values())
        self.assertFalse(scene.collected)


if __name__ == "__main__":
    unittest.main()
