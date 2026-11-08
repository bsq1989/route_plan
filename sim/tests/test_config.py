from copy import deepcopy
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

import yaml

from route_plan_sim import Scene
from route_plan_sim.config import default_robot_config
from route_plan_sim.scene import DEFAULT_CONFIG


class RobotConfigTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.folder = Path(self.directory.name)
        self.profile = yaml.safe_load(default_robot_config().read_text())

    def write_profile(self, profile=None):
        path = self.folder / "robot.yaml"
        path.write_text(yaml.safe_dump(self.profile if profile is None else profile), encoding="utf-8")
        return path

    def test_real_robot_is_the_default_for_each_new_episode(self):
        scene = Scene()
        for key, expected in dict(body_length=.5, body_width=.6, collector_width=.35,
                                  collector_depth=.24, body_margin=.1).items():
            self.assertAlmostEqual(scene.dimensions[key], expected)
        self.assertAlmostEqual(scene.safety_body.xmax - scene.safety_body.xmin, .7)
        self.assertAlmostEqual(scene.safety_body.ymax - scene.safety_body.ymin, .8)
        scene.set_dimensions(**(scene.dimensions | {"body_length": .7}))
        self.assertAlmostEqual(Scene().dimensions["body_length"], .5)

    def test_default_does_not_depend_on_working_directory(self):
        code = "from route_plan_sim import Scene; s=Scene(); assert s.dimensions['body_length']==0.5; print('ok')"
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
        self.assertAlmostEqual(scene.dimensions["body_length"], .5)
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

    def test_gap_follows_body_margin_and_lateral_offset_survives_resizing(self):
        self.profile["collector"].update(gap=.05, lateral_offset=.03)
        scene = Scene(robot_config=self.write_profile())
        scene.targets.clear()
        self.assertAlmostEqual(scene.collector.xmin, .35)
        self.assertAlmostEqual(scene.collector.ymin, -.145)
        scene.set_dimensions(**(scene.dimensions | {"body_length": .8}))
        self.assertAlmostEqual(scene.collector.xmin, .50)
        self.assertAlmostEqual(scene.collector.ymax, .205)
        scene.set_dimensions(**(scene.dimensions | {"body_margin": .15}))
        self.assertAlmostEqual(scene.collector_gap, .15)
        self.assertAlmostEqual(scene.collector.xmin, .55)

    def test_invalid_profile_never_falls_back_to_demo_sizes(self):
        profiles = []
        for section, key, value in (("body", "length", 0), ("safety", "body_margin", -.1),
                                     ("collector", "depth", float("nan")),
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
        scene = Scene()
        scene.targets.clear()
        scene.add_target(4.8, 4.8)
        scene.forward(1.5)
        self.assertIsNotNone(scene.failure)
        self.assertAlmostEqual(scene.pose[0], 4.4)  # 5 cm radius contacts sooner.
        self.assertIn((4.8, 4.8), scene.targets.values())
        self.assertFalse(scene.collected)


if __name__ == "__main__":
    unittest.main()
