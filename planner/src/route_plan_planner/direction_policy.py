"""Replaceable example policy using only stopped observations and action feedback."""

import math

from route_plan_sim.directional import MIN_DISTANCE, MIN_TURN, relative
from route_plan_sim.scene import EPS, normalize


def approach_distance(u, v, inner, tolerance):
    """Aim for the inner edge, or the closest point if it lies within the band."""
    outer = inner + tolerance
    distance = math.hypot(u, v)
    if inner - EPS <= distance <= outer + EPS:
        return 0., None
    if distance < inner or u <= 0 or abs(v) > outer + EPS:
        return None, "radial_unreachable"
    radius = max(inner, abs(v))
    travel = u - math.sqrt(max(0., radius ** 2 - v ** 2))
    if travel <= MIN_DISTANCE + EPS:
        return None, "translation_deadband"
    return travel, None


def ranked_headings(pose, points, angle, inner):
    bearings = {key: normalize(pose[2] + relative(pose, point)[1]) for key, point in points.items()}
    values = list(bearings.values())
    candidates = dict.fromkeys(values)
    for index, left in enumerate(values):
        for right in values[index + 1:]:
            gap = normalize(right - left)
            if abs(gap) <= angle + EPS:
                candidates[normalize(left + gap / 2)] = None
    ranked = []
    for yaw in candidates:
        ids = [key for key, bearing in bearings.items() if abs(normalize(bearing - yaw)) <= angle / 2 + EPS]
        reference = min(ids, key=lambda key: (math.dist(pose[:2], points[key]), key))
        distance, bearing = relative((*pose[:2], yaw), points[reference])
        forward, lateral = distance * math.cos(bearing), distance * math.sin(bearing)
        travel, failure = approach_distance(forward, lateral, inner, 0.)
        ranked.append({"yaw": yaw, "ids": ids, "reference_id": reference, "approach_m": travel,
                       "turn_rad": abs(normalize(yaw - pose[2])), "approach_reason": failure})
    return sorted(ranked, key=lambda item: (-len(item["ids"]),
                                            math.inf if item["approach_m"] is None else item["approach_m"],
                                            item["turn_rad"]))


class CoveragePolicy:
    def __init__(self, sim, plan):
        self.sim, self.plan = sim, plan
        self.memory = {}
        self.decisions, self.approaches, self.plans = [], [], []
        self.frames_seen = set()

    def capture(self):
        frame = self.sim.capture()
        for item in frame["detections"]:
            previous = self.memory.get(item["id"])
            if item["quality"] == "precise" or previous is None or previous["quality"] == "coarse":
                self.memory[item["id"]] = item
        frame["known"] = list(self.memory.values())
        self.sim.record_known(frame["index"], frame["known"])
        self.frames_seen.add(tuple(round(value, 7) for value in frame["pose"]))
        return frame

    def approach(self, reference, frame):
        sim, p = self.sim, self.sim.regions
        for _ in range(20):
            item = next((item for item in frame["detections"] if item["id"] == reference), None)
            if item is None:
                distance, reason = None, "reference_not_observed"
            else:
                radial, bearing = relative(sim.pose, item["xy_m"])
                distance, reason = approach_distance(radial * math.cos(bearing), radial * math.sin(bearing),
                                                     p.pickup_inner_radius_m, p.approach_tolerance_m)
            entry = {"reference_id": reference, "pose": sim.pose, "distance_m": distance, "reason": reason,
                     "frame_index": frame["index"]}
            self.approaches.append(entry)
            if distance is None or distance == 0:
                return frame
            if item["quality"] == "coarse":
                distance = min(distance, .40)
            entry["commanded_m"] = distance
            action = sim.forward(distance)
            if action["status"] != "success":
                entry["reason"] = action["reason"]
                return frame
            frame = self.capture()
        self.approaches.append({"reference_id": reference, "reason": "approach_limit", "pose": sim.pose})
        return frame

    def pickup(self, frame):
        sim = self.sim
        current = [item for item in frame["detections"] if sim.regions.eligible(sim.pose, item["xy_m"])]
        batch = [{"id": item["id"], "x": item["xy_m"][0], "y": item["xy_m"][1]}
                 for item in current if item["quality"] == "precise"]
        if not batch:
            return False, "no_precise_batch"
        request = sim.request(batch)
        entry = {"frame_index": frame["index"], "trace_index": len(sim.trace) - 1,
                 "request": request.to_dict(), "batch_ids": [target["id"] for target in batch]}
        try:
            response = self.plan(request)
        except ValueError as error:
            entry.update(status="rejected", reason=str(error))
        else:
            output = response.to_dict()
            # Elapsed wall time is kept for diagnostics, independent of simulated time.
            entry.update(status=response.status, reason=response.reason, response=output)
            if response.success:
                action = sim.execute_pickups(response.goals, output["pickup_stops"], entry["batch_ids"])
                entry.update(status=action["status"], reason=action["reason"])
                if action["status"] == "success":
                    for key in entry["batch_ids"]:
                        self.memory.pop(key, None)
        self.plans.append(entry)
        return entry["status"] == "success", entry["reason"] or entry["status"]

    def evaluate(self, frame, tried):
        sim, p = self.sim, self.sim.regions
        points = {key: item["xy_m"] for key, item in self.memory.items()}
        ranked = ranked_headings(sim.pose, points, math.radians(p.pickup_angle_deg), p.pickup_inner_radius_m)
        decision = {"frame_index": frame["index"], "candidates": ranked, "selected": None, "rejected": [],
                    "rule": "coverage_then_approach_then_turn"}
        self.decisions.append(decision)
        for candidate in ranked:
            start_xy = sim.pose[:2]
            marker = (round(candidate["yaw"], 6), tuple(round(v, 6) for v in start_xy))
            if marker in tried:
                continue
            tried.add(marker)
            if abs(normalize(candidate["yaw"] - sim.pose[2])) > MIN_TURN + EPS:
                action = sim.turn(candidate["yaw"])
                if action["status"] != "success":
                    decision["rejected"].append({"yaw": candidate["yaw"], "reason": action["reason"]})
                    continue
            decision["selected"] = candidate
            frame = self.approach(candidate["reference_id"], self.capture())
            success, reason = self.pickup(frame)
            if success or sim.pose[:2] != start_xy:
                return success, reason
            decision["rejected"].append({"yaw": candidate["yaw"], "reason": reason})
        rejected = decision["rejected"]
        return False, rejected[-1]["reason"] if rejected else "no_safe_heading"

    def scan(self):
        sim = self.sim
        # Recheck unresolved bearings first, then cover the remaining sectors.
        known = [normalize(sim.pose[2] + relative(sim.pose, item["xy_m"])[1])
                 for item in self.memory.values()]
        sectors = math.ceil(360 / sim.regions.view_angle_deg)
        options = known + [normalize(sim.pose[2] + i * math.tau / sectors) for i in range(sectors)]
        for yaw in options:
            marker = tuple(round(value, 7) for value in (*sim.pose[:2], yaw))
            if marker in self.frames_seen or abs(normalize(yaw - sim.pose[2])) <= MIN_TURN + EPS:
                continue
            self.frames_seen.add(marker)
            if sim.turn(yaw)["status"] == "success":
                frame = self.capture()
                # The caller may move the robot once the generator yields.
                yield frame
                if frame["pose"][:2] != sim.pose[:2]:
                    return
