import math
import unittest
from functools import partial

from route_plan_planner.api import PlanningRequest
from route_plan_planner.directional import run_case, simulate_action
from route_plan_planner.direction_policy import CoverageConfig, CoveragePolicy, approach_distance, intersect_bearings, new_view_angle, ranked_headings
from route_plan_sim.directional import DirectionalScene, load_config


class DirectionalTest(unittest.TestCase):
    def test_cluster_search_respects_density_and_remeasures_before_pickup(self):
        record = run_case("cluster-search", seed=42, cluster_radius_m=1.6)
        selections = [d for d in record["decisions"] if d["rule"] == "cluster_density"]
        self.assertEqual([len(d["candidates"]) for d in selections], [2, 2, 1])
        self.assertGreater(selections[0]["candidates"][0]["pending"], selections[0]["candidates"][1]["pending"])
        self.assertTrue(all(d["selected"] == d["candidates"][0]["id"] for d in selections))
        self.assertEqual([visit["cluster_id"] for visit in record["cluster_visits"]],
                         [d["selected"] for d in selections])
        self.assertEqual(len({d["selected"] for d in selections}), len(selections))
        observations = record["observations"]
        self.assertTrue(all(d["quality"] == "coarse" for d in observations[0]["detections"]))
        anchors = {}
        for frame in observations:
            for cluster in frame["clusters"]:
                cid, anchor = cluster["id"], cluster["anchor_xy_m"]
                self.assertEqual(anchor, anchors.setdefault(cid, anchor))
            for item in frame["known"]:
                if item["quality"] == "coarse":
                    self.assertLessEqual(math.dist(item["xy_m"], anchors[item["cluster_id"]]), 1.6 + 1e-9)
                else:
                    self.assertNotIn("cluster_id", item)
        for plan in record["plans"]:
            measured = {d["xy_m"] for frame in observations[:plan["frame_index"] + 1]
                        for d in frame["detections"] if d["quality"] == "precise"}
            self.assertTrue(all((p["x"], p["y"]) in measured for p in plan["request"]["targets"]))
        self.assertEqual(len(record["collected"]), 12)
        self.assertEqual(record["actions"][-1]["stage"], "final_scan")
        self.assertEqual(record["policy_stop_reason"], "no_observed_targets")

    def test_cluster_iterator_and_supplemental_observation(self):
        policy = CoveragePolicy(CoverageConfig(cluster_radius_m=1.6))
        self.assertFalse(policy.is_next_cluster())
        self.assertIsNone(policy.next_cluster())
        action = policy.next_action()
        pose = (0., 0., 0.)
        main = {"observation_id": 1, "detections": [{"quality": "coarse", "xy_m": (2., 0.)}]}
        supplement = [{"quality": "coarse", "xy_m": point}
                      for point in ((2.05, 0.), (0., 3.), (.4, 3.))]
        with self.assertRaises(ValueError):
            policy.submit_observation(action.action_id, pose=pose, time_s=1., observation=main,
                                      supplemental_detections=([{"quality": "precise", "xy_m": (0., 3.)}],))
        self.assertIs(policy.next_action(), action)
        self.assertTrue(policy.submit_observation(action.action_id, pose=pose, time_s=1., observation=main,
                                                  supplemental_detections=(supplement,)))
        motion = policy.next_action()
        self.assertEqual(motion.kind, "turn")
        self.assertTrue(policy.is_next_cluster())
        self.assertTrue(policy.is_next_cluster())
        cluster = policy.next_cluster()
        self.assertEqual((cluster.anchor_xy_m, cluster.radius_m, cluster.pending_count), ((0., 3.), 1.6, 2))
        self.assertFalse(policy.is_next_cluster())
        self.assertIs(policy.next_action(), motion)
        self.assertFalse(policy.submit_observation(action.action_id, pose=pose, time_s=1., observation=main))
        self.assertFalse(policy.submit_observation(motion.action_id, pose=pose, time_s=1., observation=main))
        self.assertEqual(len(policy.memory), 3)

    def test_precise_observation_interrupts_search_with_no_coarse_match(self):
        policy = CoveragePolicy(CoverageConfig(cluster_radius_m=1.6))
        first = policy.next_action()
        policy.submit_observation(
            first.action_id, pose=(0., 0., 0.), time_s=0.,
            observation={"observation_id": 1, "detections": [{"quality": "coarse", "xy_m": (1.6, 0.)}]},
        )
        motion = policy.next_action()
        cluster = policy.next_cluster()
        self.assertEqual(cluster.pending_count, 1)
        policy.submit_result(motion.action_id, status="success", pose=motion.goals[0], time_s=1.)
        observe = policy.next_action()
        points = ((1.6, 0.), (.36, 1.05))
        policy.submit_observation(
            observe.action_id, pose=motion.goals[0], time_s=2.,
            observation={"observation_id": 2, "detections": [{"quality": "precise", "xy_m": points[0]}]},
            supplemental_detections=([{"quality": "precise", "xy_m": points[1]}],),
        )
        pickup = policy.next_action()
        self.assertEqual(pickup.kind, "pickup")
        self.assertEqual({(t.x, t.y) for t in pickup.targets}, set(points))
        self.assertEqual(policy.latest_frame["clusters"][0]["pending"], 1)
        self.assertTrue(all("cluster_id" not in item for item in policy.memory.values()
                            if item["quality"] == "precise"))
        self.assertFalse(policy.is_next_cluster())

    def test_cluster_does_not_turn_again_for_hint_checked_in_current_view(self):
        policy = CoveragePolicy(CoverageConfig(cluster_radius_m=1.6))
        observe = policy.next_action()
        policy.submit_observation(
            observe.action_id, pose=(0., 0., 0.), time_s=0.,
            observation={"observation_id": 1, "detections": [
                {"quality": "coarse", "xy_m": (1.65, .1)},
                {"quality": "coarse", "xy_m": (1.7, -.12)},
            ]},
        )
        time_s = 0.
        while (action := policy.next_action()) is not None:
            self.assertLess(time_s, 12.)
            time_s += 1.
            if action.kind == "observe":
                policy.submit_observation(action.action_id, pose=policy.pose, time_s=time_s,
                                          observation={"observation_id": int(time_s) + 10, "detections": []})
            else:
                policy.submit_result(action.action_id, status="success", pose=action.goals[0], time_s=time_s)
        selected = [d for d in policy.decisions if d["rule"] == "cluster_density"]
        self.assertEqual(len(selected), 1)
        self.assertEqual(len([a for a in policy.approaches if a.get("commanded_m")]), 1)
        self.assertEqual(policy.stop_reason, "no_observed_targets")

    def test_popped_cluster_stays_out_of_queue_after_rediscovery(self):
        policy = CoveragePolicy(CoverageConfig(cluster_radius_m=1.6))
        pose = (0., 0., 0.)
        point = {"quality": "coarse", "xy_m": (2., 0.)}
        first = policy.next_action()
        policy.submit_observation(first.action_id, pose=pose, time_s=0.,
                                  observation={"observation_id": 1, "detections": [point]})
        selected = policy.next_cluster()
        for time_s in (1., 2.):
            step = policy.next_action()
            self.assertEqual(step.kind, "forward")
            policy.submit_result(step.action_id, status="blocked", pose=pose, time_s=time_s)
        turn = policy.next_action()
        self.assertEqual(turn.kind, "turn")
        policy.submit_result(turn.action_id, status="success", pose=turn.goals[0], time_s=3.)
        observe = policy.next_action()
        policy.submit_observation(observe.action_id, pose=turn.goals[0], time_s=4.,
                                  observation={"observation_id": 2, "detections": [point]})
        self.assertFalse(policy.is_next_cluster())
        self.assertIn(selected.cluster_id, {hint["cluster_id"] for hint in policy.memory.values()})

    def test_cluster_search_retains_hints_when_motion_is_blocked(self):
        record = run_case("blocked", seed=42, cluster_radius_m=1.6)
        self.assertEqual(record["policy_stop_reason"], "incomplete_final_scan")
        self.assertTrue(record["unresolved_observations"])
        self.assertTrue(any(action["status"] == "blocked" for action in record["actions"]))
        self.assertIsNone(record["failure"])

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
        config["custom"]["directional"]["cases"]["changing-ids"] = {"targets": [[2.45, 3.1]]}
        sim = DirectionalScene(config, "changing-ids", 42, planner_request=PlanningRequest.from_scene)
        policy = CoveragePolicy(use_deferred_bearings=True)
        observe = policy.next_action()
        first = sim.capture()
        policy.submit_result(observe.action_id, status="success", pose=sim.pose, time_s=sim.time,
                             observation={**first, "observation_id": 0})
        action = policy.next_action()
        self.assertEqual(action.kind, "turn")
        simulate_action(policy, sim, action)
        observe = policy.next_action()
        self.assertEqual(observe.kind, "observe")
        previous = policy.latest_frame["detections"][0]
        simulate_action(policy, sim, observe)
        action = policy.next_action()
        self.assertEqual(action.kind, "forward")
        refreshed = policy.latest_frame["detections"][0]
        self.assertNotEqual(previous["id"], refreshed["id"])
        simulate_action(policy, sim, action)
        simulate_action(policy, sim, policy.next_action())
        self.assertIn(refreshed["id"], {item["id"] for item in policy.latest_frame["deferred"]})

    def test_gateway_waits_for_matching_fresh_feedback(self):
        policy = CoveragePolicy()
        pose = (0., 0., 0.)
        observe = policy.next_action()
        self.assertIs(observe, policy.next_action())
        observation = {"observation_id": "frame-1", "detections": [{
            "quality": "bearing", "origin_xy_m": [.1, 0.], "bearing_map_rad": 0.,
        }]}
        self.assertTrue(policy.submit_result(observe.action_id, status="success", pose=pose,
                                             time_s=1., observation=observation))
        motion = policy.next_action()
        self.assertEqual(motion.kind, "forward")
        self.assertFalse(policy.submit_result(observe.action_id, status="success", pose=pose, time_s=1.))
        self.assertIs(motion, policy.next_action())
        actual = (.20, 0., 0.)
        self.assertTrue(policy.submit_result(motion.action_id, status="blocked", pose=actual, time_s=2.))
        observe = policy.next_action()
        self.assertEqual(observe.kind, "observe")
        self.assertEqual(observe.start_pose, actual)
        with self.assertRaisesRegex(ValueError, "previous result"):
            policy.submit_result(observe.action_id, status="success", pose=actual, time_s=3.,
                                 observation={**observation, "observation_id": "frame-2", "time_s": 1.5})
        self.assertIs(observe, policy.next_action())
        policy.submit_result(observe.action_id, status="cancelled", pose=actual, time_s=3.)
        self.assertIsNone(policy.next_action())
        self.assertEqual(policy.stop_reason, "cancelled")

    def test_partial_gateway_pickup_preserves_unconfirmed_target(self):
        policy = CoveragePolicy()
        first, second = (1.0, 0.), (1.2, .25)
        observe = policy.next_action()
        policy.submit_result(observe.action_id, status="success", pose=[0, 0, 0], time_s=1.,
                             observation={"observation_id": 1, "detections": [
                                 {"quality": "precise", "xy_m": point} for point in (first, second)
                             ]})
        action = policy.next_action()
        self.assertEqual(action.kind, "pickup")
        targets = {(target.x, target.y): target.id for target in action.targets}
        self.assertEqual(set(targets), {first, second})
        with self.assertRaisesRegex(ValueError, "all requested IDs"):
            policy.submit_result(action.action_id, status="success", pose=action.start_pose,
                                 time_s=2., picked_ids=[targets[first]])
        policy.submit_result(action.action_id, status="partial", pose=action.goals[0],
                             time_s=2., picked_ids=[targets[first]])
        observe = policy.next_action()
        self.assertEqual(observe.kind, "observe")
        self.assertEqual(policy.local_points(), [second])
        policy.submit_result(observe.action_id, status="success", pose=action.goals[0], time_s=3.,
                             observation={"observation_id": 2, "detections": []})
        retry = policy.next_action()
        self.assertEqual(retry.kind, "pickup")
        self.assertEqual([(target.x, target.y) for target in retry.targets], [second])

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
        policy = CoveragePolicy(request_options=sim.request([]).to_dict())
        first = policy.next_action()
        frame = sim.capture()
        closest = frame["detections"][0]["xy_m"]
        self.assertFalse(sim.regions.eligible(sim.pose, closest))
        policy.submit_result(first.action_id, status="success", pose=sim.pose, time_s=sim.time,
                             observation={**frame, "observation_id": frame["index"]})
        action = policy.next_action()
        self.assertEqual(action.kind, "pickup")
        self.assertEqual({(t.x, t.y) for t in action.targets}, {tuple(xy) for xy in config["custom"]["directional"]["cases"]["right-cluster"]["targets"][:2]})
        while not sim.all_collected and (action := policy.next_action()) is not None:
            simulate_action(policy, sim, action)
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
                policy = CoveragePolicy(CoverageConfig(view_angle_deg=sim.regions.view_angle_deg),
                                        request_options=sim.request([]).to_dict())
                simulate_action(policy, sim, policy.next_action())
                hint, = policy.latest_frame["detections"]
                self.assertEqual(hint, {"id": 1, "quality": "bearing", "origin_xy_m": (.8, 2.1),
                                        "bearing_map_rad": 0.})
                start = sim.pose
                action = policy.next_action()
                self.assertEqual(action.kind, "forward")
                self.assertLessEqual(action.distance_m, .40)
                simulate_action(policy, sim, action)
                if name == "distant":
                    while (action := policy.next_action()) is not None and action.kind != "pickup":
                        simulate_action(policy, sim, action)
                    self.assertIsNotNone(action)
                    updated = policy.latest_frame
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
