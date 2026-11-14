import math
import unittest

from route_plan_planner.api import PlanningRequest
from route_plan_planner.directional import run_case
from route_plan_planner.direction_policy import approach_distance, ranked_headings
from route_plan_sim.directional import DirectionalScene, load_config


class DirectionalTest(unittest.TestCase):
    def test_wraparound_and_radial_approach(self):
        points = {1: (-1, .1), 2: (-1, -.1), 3: (1, 0)}
        self.assertEqual(set(ranked_headings((0, 0, 0), points, math.pi / 3, .75)[0]["ids"]), {1, 2})
        distance, reason = approach_distance(1.3, .3, .75, .05)
        self.assertIsNone(reason)
        self.assertAlmostEqual(distance, .612613645756624)
        self.assertEqual(approach_distance(1.3, .85, .75, .05)[1], "radial_unreachable")

    def test_replays_full_pickup_and_rejects_unsafe_plans(self):
        records = {name: run_case(name, seed=42) for name in
                   ("two-clusters", "occlusion", "blocked", "fence-limit", "coarse-only")}
        full = records["two-clusters"]
        self.assertEqual(full["stop_reason"], "all_collected")
        self.assertGreaterEqual(full["metrics"]["pickup_stops"], 2)
        self.assertFalse(full["remaining"])
        for record in records.values():
            for frame in record["trace"]:
                if circle := frame["auto_fence"]:
                    self.assertLessEqual(math.dist(frame["pose"][:2], circle[:2]), circle[2] + 1e-9)
        self.assertEqual(records["occlusion"]["stop_reason"], "all_collected")
        self.assertEqual([item["id"] for item in records["occlusion"]["observations"][0]["detections"]], [1])
        self.assertEqual([plan["batch_ids"] for plan in records["occlusion"]["plans"]], [[1], [2]])
        self.assertTrue(records["blocked"]["remaining"])
        self.assertTrue(any(action["status"] == "blocked" for action in records["blocked"]["actions"]))
        self.assertTrue(records["fence-limit"]["remaining"])
        self.assertTrue(records["coarse-only"]["remaining"])
        self.assertEqual(full["plans"][0]["batch_ids"], [1, 2, 3])
        for entry in full["plans"]:
            frame = full["observations"][entry["frame_index"]]
            self.assertEqual(entry["request"]["pose"], list(frame["pose"]))
            observed = {item["id"] for item in frame["detections"] if item["quality"] == "precise"}
            self.assertLessEqual(set(entry["batch_ids"]), observed)

    def test_radial_band_boundaries_and_outside_targets(self):
        record = run_case("near-mid-far")
        self.assertEqual([entry["batch_ids"] for entry in record["plans"]], [[1, 2], [3], [4]])
        sim = DirectionalScene(load_config(), "offset", 42, planner_request=PlanningRequest.from_scene)
        for radius, angle in ((.75, 0), (1.4, math.pi / 6)):
            self.assertTrue(sim.regions.eligible((0, 0, 0), (radius * math.cos(angle), radius * math.sin(angle))))
        self.assertFalse(sim.regions.eligible((0, 0, 0), (.749, 0)))
        self.assertFalse(sim.regions.eligible((0, 0, 0), (1.401, 0)))

    def test_full_truth_rejects_unrequested_pickup_and_out_of_circle_route(self):
        config = load_config()
        cases = config["custom"]["directional"]["cases"]
        cases["conflict"] = {"targets": [[1.53, 2.1], [1.54, 2.15]]}
        sim = DirectionalScene(config, "conflict", 42, planner_request=PlanningRequest.from_scene)
        before = sim.final_state()
        stop = {"goal_index": 0, "target_ids": [1]}
        self.assertEqual(sim.execute_pickups([(0.95, 2.1, 0)], [stop], [1])["reason"], "pickup_conflict")
        self.assertEqual(sim.final_state(), before)
        sim = DirectionalScene(config, "fence-limit", 42, planner_request=PlanningRequest.from_scene)
        before = sim.final_state()
        action = sim.execute_pickups([(0.88, 2.1, 0), (1.05, 2.1, 0)], [], [1])
        self.assertEqual(action["reason"], "Motion circle boundary")
        self.assertEqual(sim.final_state(), before)
        self.assertEqual(len(sim.trace), 1)
