import math
import unittest
from functools import partial

from route_plan_planner.api import PlanningRequest, plan
from route_plan_planner.directional import run_case
from route_plan_planner.direction_policy import CoveragePolicy, approach_distance, intersect_bearings, new_view_angle, ranked_headings
from route_plan_sim.directional import DirectionalScene, load_config


class DirectionalTest(unittest.TestCase):
    def test_interior_random_case_keeps_turn_footprint_away_from_walls(self):
        config = load_config()
        sim = DirectionalScene(config, "random-40-interior", 42, planner_request=PlanningRequest.from_scene)
        points = sim.initial["remaining"].values()
        self.assertEqual(len(points), 40)
        # Default collector corner determines the sweep, plus the target radius.
        clearance = math.hypot(.624, .095) + .05
        for x, y in points:
            self.assertGreater(min(x, y, 12 - x, 9 - y), clearance)

    def test_wraparound_and_radial_approach(self):
        points = {1: (-1, .1), 2: (-1, -.1), 3: (1, 0)}
        self.assertEqual(set(ranked_headings((0, 0, 0), points, math.pi / 3, .75)[0]["ids"]), {1, 2})
        distance, reason = approach_distance(1.3, .3, .75, .05)
        self.assertIsNone(reason)
        self.assertAlmostEqual(distance, .612613645756624)
        self.assertEqual(approach_distance(1.3, .85, .75, .05)[1], "radial_unreachable")

    def test_scan_counts_only_new_angular_coverage(self):
        angle = math.pi / 2
        self.assertAlmostEqual(new_view_angle(math.pi, [-math.pi], angle), 0.)
        self.assertAlmostEqual(new_view_angle(0., [-angle / 2, angle / 2], angle), 0.)
        self.assertAlmostEqual(new_view_angle(angle / 2, [0.], angle), angle / 2)

    def test_six_forward_bearings_rank_ahead_of_one_precise_target_behind(self):
        points = {1: (-.8, .5)}
        bearings = {key: math.radians(yaw) for key, yaw in enumerate((-25, -20, -15, -10, 0, 10), 2)}
        ranked = ranked_headings((0., 0., 0.), points, math.pi / 3, .75, bearings)
        self.assertEqual(set(ranked[0]["ids"]), set(bearings))
        self.assertIn(ranked[0]["reference_id"], bearings)
        self.assertAlmostEqual(ranked[0]["yaw"], 0.)

    def test_heading_score_accounts_for_approach_distance(self):
        points = {1: (4., 0.), 2: (0., 1.5)}
        ranked = ranked_headings((0., 0., 0.), points, math.pi / 3, .75, precision_range=1.4)
        self.assertEqual(ranked[0]["reference_id"], 2)
        self.assertLess(ranked[0]["cost"], ranked[1]["cost"])

    def test_side_bearing_is_measured_and_picked_on_first_pass(self):
        record = run_case("random-40-interior", seed=42)
        point = record["initial"]["remaining"][31]
        first_measurement = next(f for f in record["observations"]
                                 if any(d.get("xy_m") == point for d in f["detections"]))
        probe = next(a for a in record["approaches"]
                     if a.get("source") == "bearing_leaving_view" and a["frame_index"] == first_measurement["index"] - 1)
        self.assertLessEqual(math.dist(probe["pose"][:2], point), 1.4)
        self.assertEqual(probe["pose"][:2], first_measurement["pose"][:2])
        stop = next(stop for action in record["actions"] for stop in action.get("pickups", ()) if 31 in stop["ids"])
        self.assertLess(record["trace"][stop["trace_index"]]["time_s"], 12.)
        self.assertIsNone(record["failure"])

    def test_bearing_crossing_requires_separated_directions_and_valid_ranges(self):
        first = {"origin_xy_m": (0., 0.), "bearing_map_rad": 0.}
        second = {"origin_xy_m": (1., 1.), "bearing_map_rad": math.atan2(-1., 2.)}
        point = intersect_bearings(first, second, 1.4, 5.)
        self.assertAlmostEqual(point[0], 3.)
        self.assertAlmostEqual(point[1], 0.)
        self.assertIsNone(intersect_bearings(first, second, 2.5, 5.))
        self.assertIsNone(intersect_bearings(first, {**second, "bearing_map_rad": .01}, 1.4, 5.))

    def test_nearby_bearings_reduce_return_trips_without_entering_plan(self):
        record = run_case("random-40-interior", seed=42)
        stops = {key: record["trace"][stop["trace_index"]]["time_s"] for action in record["actions"]
                 for stop in action.get("pickups", ()) for key in stop["ids"]}
        self.assertEqual(record["stop_reason"], "all_collected")
        self.assertLess(stops[30], 30.)
        self.assertLess(stops[32], 35.)
        measured = set()
        plan_frames = iter(record["plans"])
        entry = next(plan_frames, None)
        for frame in record["observations"]:
            measured.update(item["xy_m"] for item in frame["detections"] if item["quality"] == "precise")
            while entry is not None and entry["frame_index"] == frame["index"]:
                self.assertTrue(all((target["x"], target["y"]) in measured for target in entry["request"]["targets"]))
                entry = next(plan_frames, None)
        self.assertIsNone(entry)

    def test_observation_memory_survives_detection_id_changes(self):
        config = load_config()
        config["custom"]["directional"]["cases"]["changing-ids"] = {
            "targets": [[1.6, 2.1], [2.45, 3.1]],
        }
        sim = DirectionalScene(config, "changing-ids", 42, planner_request=PlanningRequest.from_scene)
        policy = CoveragePolicy(sim, None, use_deferred_bearings=True)
        first, second = policy.capture(), policy.capture()
        self.assertEqual([d["quality"] for d in first["detections"]], ["precise", "bearing"])
        self.assertTrue({d["id"] for d in first["detections"]}.isdisjoint(d["id"] for d in second["detections"]))
        self.assertEqual(first["known"][0]["id"], second["known"][0]["id"])
        self.assertEqual(len(second["known"]), 2)
        self.assertEqual(second["known"][1]["id"], second["detections"][1]["id"])
        stale = sim.execute_pickups([], [], [first["detections"][0]["id"]], frame_index=first["index"])
        self.assertEqual(stale["reason"], "stale_or_invalid_detection")
        self.assertEqual(sim.forward(.25)["status"], "success")
        third = policy.capture()
        self.assertEqual(third["known"][0]["id"], first["known"][0]["id"])
        self.assertEqual(third["known"][1]["origin_xy_m"], sim.pose[:2])
        self.assertEqual([item["id"] for item in third["deferred"]], [second["detections"][1]["id"]])

    def test_deferred_bearings_trigger_fresh_observation_before_scan(self):
        baseline = run_case("random-40-interior", seed=42)
        record = run_case("random-40-interior", seed=42,
                          policy=partial(CoveragePolicy, use_deferred_bearings=True))
        probe = next(a for a in record["approaches"] if a.get("source") == "deferred_bearing")
        before, after = (record["observations"][probe[key]] for key in ("frame_index", "observation_index"))
        self.assertEqual(before["pose"][:2], after["pose"][:2])
        self.assertTrue(any(item["origin_xy_m"] == probe["origin_xy_m"]
                            and item["bearing_map_rad"] == probe["bearing_map_rad"] for item in before["deferred"]))
        self.assertTrue(after["detections"])
        self.assertFalse(any(f["deferred"] for f in baseline["observations"]))
        for key in (24, 37):
            times = [next(r["trace"][stop["trace_index"]]["time_s"] for action in r["actions"]
                          for stop in action.get("pickups", ()) if key in stop["ids"]) for r in (record, baseline)]
            self.assertLess(*times)
        self.assertEqual(record["stop_reason"], "all_collected")

    def test_replays_full_pickup_and_rejects_unsafe_plans(self):
        records = {name: run_case(name, seed=42) for name in
                   ("two-clusters", "occlusion", "blocked", "fence-limit", "coarse-only")}
        full = records["two-clusters"]
        self.assertEqual(full["stop_reason"], "all_collected")
        self.assertGreaterEqual(full["metrics"]["pickup_stops"], 2)
        self.assertFalse(full["remaining"])
        self.assertEqual(full["trace"][-1]["stage"], "pickup_stop")
        for record in records.values():
            for frame in record["trace"]:
                if circle := frame["auto_fence"]:
                    self.assertLessEqual(math.dist(frame["pose"][:2], circle[:2]), circle[2] + 1e-9)
        self.assertEqual(records["occlusion"]["stop_reason"], "all_collected")
        self.assertEqual([item["id"] for item in records["occlusion"]["observations"][0]["detections"]], [1])
        self.assertEqual([stop["ids"] for action in records["occlusion"]["actions"]
                          for stop in action.get("pickups", ())], [[1], [2]])
        self.assertFalse(records["occlusion"]["unresolved_observations"])
        self.assertTrue(records["blocked"]["remaining"])
        self.assertTrue(any(action["status"] == "blocked" for action in records["blocked"]["actions"]))
        self.assertTrue(records["fence-limit"]["remaining"])
        self.assertTrue(records["coarse-only"]["remaining"])
        first_batch = [(target["x"], target["y"]) for target in full["plans"][0]["request"]["targets"]]
        self.assertEqual(first_batch, [full["initial"]["remaining"][key] for key in (3, 4)])
        self.assertEqual(full["actions"][0]["stage"], "pickup")
        for entry in full["plans"]:
            frame = full["observations"][entry["frame_index"]]
            self.assertEqual(entry["request"]["pose"], list(frame["pose"]))
            measured = {item["xy_m"] for prior in full["observations"][:frame["index"] + 1]
                        for item in prior["detections"] if item["quality"] == "precise"}
            for target in entry["request"]["targets"]:
                point = target["x"], target["y"]
                self.assertIn(point, measured)
                self.assertLessEqual(math.dist(point, frame["pose"][:2]), 1.4 + 1e-9)

    def test_local_batch_includes_measured_points_outside_current_view_and_annulus(self):
        config = load_config()
        config["custom"]["directional"]["cases"]["right-cluster"] = {
            "world": {"width": 12, "height": 9}, "pose": [9.225, 5.620, math.radians(58.4)],
            "targets": [[9.609, 6.244], [9.624, 6.522], [10.046, 6.929]],
        }
        sim = DirectionalScene(config, "right-cluster", 42, planner_request=PlanningRequest.from_scene)
        policy = CoveragePolicy(sim, plan)
        first = policy.capture()
        closest = first["detections"][0]["xy_m"]
        self.assertFalse(sim.regions.eligible(sim.pose, closest))
        self.assertEqual(sim.turn(0.)["status"], "success")
        frame = policy.capture()
        self.assertFalse(frame["detections"])
        self.assertEqual(policy.evaluate(frame, set()), (True, "success"))
        self.assertEqual(set(sim.final_state()["collected"]), {1, 2})
        self.assertFalse(policy.local_points())
        frame = policy.capture()
        self.assertEqual(policy.evaluate(frame, set()), (True, "success"))
        self.assertTrue(sim.all_collected)

    def test_bearing_only_approach_remeasures_and_checks_hidden_obstacle(self):
        config = load_config()
        cases = config["custom"]["directional"]["cases"]
        cases["distant"] = {"world": {"width": 7}, "targets": [[5.2, 2.1]]}
        # The blocker is in the vehicle's sweep, outside the narrower camera view.
        cases["distant-blocked"] = {**cases["distant"], "view_angle_deg": 10,
                                    "targets": [[5.2, 2.1], [1.45, 1.98]]}
        for name in ("distant", "distant-blocked"):
            with self.subTest(name=name):
                sim = DirectionalScene(config, name, 42, planner_request=PlanningRequest.from_scene)
                policy = CoveragePolicy(sim, None)
                frame = policy.capture()
                hint = next(item for item in frame["detections"] if item["id"] == 1)
                self.assertEqual(hint, {"id": 1, "quality": "bearing", "origin_xy_m": (.8, 2.1),
                                        "bearing_map_rad": 0.})
                start = sim.pose
                self.assertEqual(policy.pickup(frame), (False, "no_precise_batch"))
                updated = policy.approach(hint, frame)
                self.assertEqual(policy.approaches[0]["source"], "bearing")
                self.assertTrue(all(item["commanded_m"] <= .40 for item in policy.approaches))
                if name == "distant":
                    self.assertNotEqual(updated["detections"][0]["id"], hint["id"])
                    self.assertEqual(updated["detections"][0]["quality"], "precise")
                    self.assertEqual(updated["detections"][0]["xy_m"], (5.2, 2.1))
                    self.assertLessEqual(math.dist(sim.pose[:2], (5.2, 2.1)), sim.regions.precision_range_m)
                    self.assertEqual(len(sim.observations), len(sim.actions) + 1)
                else:
                    self.assertEqual(sim.pose, start)
                    self.assertEqual(sim.actions[-1]["reason"], "Body hit target #2")
                self.assertFalse(sim.final_state()["collected"])

    def test_radial_band_boundaries_and_outside_targets(self):
        record = run_case("near-mid-far")
        points = record["initial"]["remaining"]
        batches = [[(p["x"], p["y"]) for p in entry["request"]["targets"]] for entry in record["plans"]]
        self.assertEqual(batches, [[points[1], points[2]], [points[3]], [points[4]]])
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
