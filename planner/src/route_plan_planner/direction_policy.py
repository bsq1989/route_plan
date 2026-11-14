"""Replaceable example policy using only stopped observations and action feedback."""

import math

from route_plan_sim.directional import MIN_DISTANCE, MIN_TURN, relative
from route_plan_sim.scene import EPS, normalize


BEARING_STEP_M = .40
TRAVEL_COST_WEIGHT = .5
TURN_COST_WEIGHT = .5
MAX_SIDE_PROBE_TURN = math.radians(12)
NEAR_OBSERVATION_REACH = 2 * BEARING_STEP_M
BEARING_HISTORY_STOPS = 4
MIN_BEARING_CROSSING_SINE = math.sin(math.radians(4))
DEFERRED_BEARING_LIMIT = 96
DEFERRED_BEARING_LIFETIME_S = 30.
DEFERRED_RECHECK_DISTANCE = NEAR_OBSERVATION_REACH


def same_position(a, b):
    return math.dist(a, b) <= EPS


def new_view_angle(yaw, seen, width):
    """Angular coverage added by a frame at the same observation position."""
    half = width / 2
    intervals = []
    for heading in seen:
        delta = normalize(heading - yaw)
        for offset in (-math.tau, 0., math.tau):
            left, right = max(-half, delta + offset - half), min(half, delta + offset + half)
            if left < right:
                intervals.append((left, right))
    covered, edge = 0., -half
    for left, right in sorted(intervals):
        if right > edge:
            covered += right - max(left, edge)
            edge = right
    return max(0., width - covered)


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


def ranked_headings(pose, points, angle, inner, bearing_hints=None, *, precision_range=1.4):
    bearings = dict(bearing_hints or {})
    bearings.update({key: normalize(pose[2] + relative(pose, point)[1]) for key, point in points.items()})
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
        located = [key for key in ids if key in points]
        actual = located or [key for key in ids if abs(normalize(yaw - bearings[key])) <= MIN_TURN + EPS]
        if not actual:
            # A bearing-only approach starts along an observed ray.
            continue
        reference = min(actual, key=lambda key: (math.dist(pose[:2], points[key]) if key in points else 0., key))
        travel, failure = None, None
        if reference in points:
            distance, bearing = relative((*pose[:2], yaw), points[reference])
            forward, lateral = distance * math.cos(bearing), distance * math.sin(bearing)
            travel, failure = approach_distance(forward, lateral, inner, 0.)
        turn = abs(normalize(yaw - pose[2]))
        distance = math.dist(pose[:2], points[reference]) if reference in points else None
        # Bearing-only detections carry no distance. Charge the next commanded
        # step, and use a measured range only when the memory has a position.
        approach = (max(0., distance - precision_range) if distance is not None else BEARING_STEP_M)
        cost = TRAVEL_COST_WEIGHT * approach / precision_range + TURN_COST_WEIGHT * turn / math.pi
        ranked.append({"yaw": yaw, "ids": ids, "reference_id": reference, "approach_m": travel,
                       "turn_rad": turn, "approach_reason": failure, "cost": cost, "score": len(ids) - cost})
    return sorted(ranked, key=lambda item: (-item["score"], item["reference_id"] not in points, item["turn_rad"]))


def near_bearings_leaving_view(bearings, current, precision_range):
    """Estimate a bearing interval from the part of a previous ray now in range."""
    x, y, heading = current["pose"]
    half_view = math.radians(current["view_angle_deg"] / 2)
    for detection in bearings:
        ox, oy = detection["origin_xy_m"]
        angle = detection["bearing_map_rad"]
        c, s = math.cos(angle), math.sin(angle)
        forward, lateral = (x - ox) * c + (y - oy) * s, (y - oy) * c - (x - ox) * s
        if abs(lateral) >= precision_range:
            continue
        reach = math.sqrt(precision_range**2 - lateral**2)
        start = max(precision_range, forward - reach)
        end = min(current["view_range_m"], forward + reach)
        if end <= start + EPS:
            continue
        a, b = (normalize(math.atan2(oy + length * s - y, ox + length * c - x) - heading)
                for length in (start, end))
        lo, hi = sorted((a, a + normalize(b - a)))
        overlaps_view = any(lo <= half_view + shift + EPS and hi >= -half_view + shift - EPS
                            for shift in (-math.tau, 0., math.tau))
        if not overlaps_view:
            if (lo + hi) / 2 > math.pi:
                lo, hi = lo - math.tau, hi - math.tau
            elif (lo + hi) / 2 < -math.pi:
                lo, hi = lo + math.tau, hi + math.tau
            yield lo, hi, detection


def intersect_bearings(first, second, precision_range, view_range):
    a, b = first["bearing_map_rad"], second["bearing_map_rad"]
    denominator = math.sin(b - a)
    if abs(denominator) < MIN_BEARING_CROSSING_SINE:
        return None
    x, y = first["origin_xy_m"]
    ox, oy = second["origin_xy_m"]
    dx, dy = ox - x, oy - y
    first_range = (dx * math.sin(b) - dy * math.cos(b)) / denominator
    second_range = (dx * math.sin(a) - dy * math.cos(a)) / denominator
    if all(precision_range < distance <= view_range + EPS for distance in (first_range, second_range)):
        return x + first_range * math.cos(a), y + first_range * math.sin(a)
    return None


class CoveragePolicy:
    def __init__(self, sim, plan, *, use_deferred_bearings=False):
        self.sim, self.plan = sim, plan
        self.use_deferred_bearings = use_deferred_bearings
        self.memory = {}
        self.decisions, self.approaches, self.plans = [], [], []
        self.sweep_origin = None
        self.seen_headings = []
        self.next_point_id = -1
        self.last_pickup_attempt = None
        self.resume_heading = None
        self.recent_bearings = []
        self.observation_origin = None
        self.observation_attempts = set()
        self.deferred_bearings = {}
        self.deferred_probe_pose = None
        self.deferred_misses = set()
        self.latest_frame = None

    def capture(self):
        frame = self.sim.capture()
        x, y, heading = frame["pose"]
        half_view = math.radians(frame["view_angle_deg"] / 2)
        bearings = [item for item in frame["detections"] if item["quality"] == "bearing"]
        current_keys = {self.bearing_key(ray) for ray in bearings}
        for key, item in list(self.memory.items()):
            if item["quality"] == "bearing" and (
                item["origin_xy_m"] != (x, y)
                or abs(normalize(item["bearing_map_rad"] - heading)) <= half_view + EPS
            ):
                if self.bearing_key(item) not in current_keys:
                    self.defer_bearing(item)
                del self.memory[key]
        available = {key for key, item in self.memory.items() if item["quality"] == "precise"}
        for item in frame["detections"]:
            key = item["id"]
            if item["quality"] == "precise":
                key = next((key for key in self.memory if key in available
                            and same_position(self.memory[key]["xy_m"], item["xy_m"])), None)
                if key is None:
                    key = self.next_point_id
                    self.next_point_id -= 1
                available.discard(key)
            self.memory[key] = {**item, "id": key}
            if item["quality"] == "bearing":
                self.memory[key]["observed_s"] = frame["time_s"]
        for key in current_keys:
            self.deferred_bearings.pop(key, None)
            self.deferred_misses.discard(key)
        self.deferred_bearings = {key: item for key, item in self.deferred_bearings.items()
                                  if frame["time_s"] - item["observed_s"] <= DEFERRED_BEARING_LIFETIME_S}
        self.deferred_misses.intersection_update(self.deferred_bearings)
        if bearings:
            if self.recent_bearings and self.recent_bearings[-1][0] == (x, y):
                rays = {item["bearing_map_rad"]: item for item in self.recent_bearings[-1][1]}
                rays.update({item["bearing_map_rad"]: item for item in bearings})
                self.recent_bearings[-1] = ((x, y), list(rays.values()))
            else:
                self.recent_bearings.append(((x, y), bearings))
                self.recent_bearings = self.recent_bearings[-BEARING_HISTORY_STOPS:]
        frame["known"] = list(self.memory.values())
        frame["deferred"] = list(self.deferred_bearings.values())
        self.sim.record_known(frame["index"], frame["known"], deferred=frame["deferred"])
        self.latest_frame = frame
        if frame["pose"][:2] != self.sweep_origin:
            self.sweep_origin = frame["pose"][:2]
            self.seen_headings.clear()
        if heading not in self.seen_headings:
            self.seen_headings.append(heading)
        return frame

    @staticmethod
    def bearing_key(item):
        return item["origin_xy_m"], item["bearing_map_rad"]

    def defer_bearing(self, item):
        if not self.use_deferred_bearings:
            return
        key = self.bearing_key(item)
        self.deferred_bearings[key] = item
        if len(self.deferred_bearings) > DEFERRED_BEARING_LIMIT:
            oldest = min(self.deferred_bearings, key=lambda key: self.deferred_bearings[key]["observed_s"])
            del self.deferred_bearings[oldest]

    def approach(self, hint, frame):
        sim, p = self.sim, self.sim.regions
        reference = hint["id"]
        missed_bearings = []
        for _ in range(20):
            if hint["quality"] == "precise":
                item = next((item for item in frame["detections"]
                             if item["quality"] == "precise" and same_position(item["xy_m"], hint["xy_m"])), None)
            else:
                item = min((item for item in frame["detections"] if item["quality"] == "bearing"
                            and abs(normalize(item["bearing_map_rad"] - sim.pose[2])) <= MIN_TURN + EPS),
                           key=lambda item: abs(normalize(item["bearing_map_rad"] - sim.pose[2])), default=None)
            entry = {"reference_id": reference, "pose": sim.pose, "frame_index": frame["index"]}
            if item is None:
                distance, reason = None, "reference_not_observed"
            elif item["quality"] == "bearing":
                entry["source"] = "bearing"
                distance, reason = BEARING_STEP_M, None
            else:
                radial, bearing = relative(sim.pose, item["xy_m"])
                distance, reason = approach_distance(radial * math.cos(bearing), radial * math.sin(bearing),
                                                     p.pickup_inner_radius_m, p.approach_tolerance_m)
            entry.update(distance_m=distance, reason=reason)
            self.approaches.append(entry)
            if distance is None or distance == 0:
                return frame
            entry["commanded_m"] = distance
            action = sim.forward(distance)
            if action["status"] != "success":
                entry["reason"] = action["reason"]
                return frame
            previous, frame = frame, self.capture()
            if self.local_points():
                return frame
            if item["quality"] == "bearing":
                bearings = [item for item in previous["detections"] if item["quality"] == "bearing"]
                frame, missed_bearings = self.probe_side_bearings(bearings, missed_bearings, frame)
                if frame["pose"][2] != previous["pose"][2]:
                    return frame
        self.approaches.append({"reference_id": reference, "reason": "approach_limit", "pose": sim.pose})
        return frame

    def probe_side_bearings(self, bearings, previous_misses, frame):
        # The projected rays choose where to look. Only capture() can produce a
        # measured point for pickup, and turn() checks the entire swept body.
        p = self.sim.regions
        fresh = list(near_bearings_leaving_view(bearings, frame, p.precision_range_m))
        intervals = fresh + list(near_bearings_leaving_view(previous_misses, frame, p.precision_range_m))
        if not intervals:
            return frame, []
        missed = [item for _, _, item in fresh]
        half_view = math.radians(p.view_angle_deg / 2)
        heading = frame["pose"][2]
        offsets = {max(MIN_TURN * 2, hi - half_view + MIN_TURN) if lo > 0
                   else min(-MIN_TURN * 2, lo + half_view - MIN_TURN)
                   for lo, hi, _ in intervals}
        offsets = {offset for offset in offsets if abs(offset) <= MAX_SIDE_PROBE_TURN}
        if not offsets:
            return frame, missed
        offset = min(offsets, key=lambda offset: (abs(offset) / math.pi - sum(
            offset - half_view <= lo <= hi <= offset + half_view for lo, hi, _ in intervals), abs(offset), offset))
        yaw = normalize(heading + offset)
        action = self.sim.turn(yaw)
        self.approaches.append({"pose": frame["pose"], "frame_index": frame["index"], "source": "bearing_leaving_view",
                                "yaw_rad": yaw, "reason": action["reason"]})
        if action["status"] != "success":
            return frame, missed
        frame = self.capture()
        if self.local_points():
            if self.resume_heading is None:
                self.resume_heading = heading
            return frame, []
        if self.sim.turn(heading)["status"] == "success":
            frame = self.capture()
        return frame, missed

    def local_points(self):
        sim = self.sim
        radius = sim.regions.precision_range_m
        return [item["xy_m"] for item in self.memory.values()
                if item["quality"] == "precise" and math.dist(sim.pose[:2], item["xy_m"]) <= radius + EPS]

    def recheck_deferred(self):
        """Look toward old rays from this pose, and let fresh detections drive any move."""
        sim, p = self.sim, self.sim.regions
        pose = sim.pose
        if not self.deferred_bearings or self.deferred_probe_pose == pose[:2]:
            return None
        radius = p.precision_range_m + DEFERRED_RECHECK_DISTANCE
        half_view = math.radians(p.view_angle_deg / 2)
        width = 2 * half_view
        active = [item["bearing_map_rad"] for item in self.memory.values() if item["quality"] == "bearing"]
        options = []
        for key, item in self.deferred_bearings.items():
            origin, bearing = key
            if origin == pose[:2] or key in self.deferred_misses:
                continue
            c, s = math.cos(bearing), math.sin(bearing)
            dx, dy = pose[0] - origin[0], pose[1] - origin[1]
            along, across = dx * c + dy * s, dy * c - dx * s
            if abs(across) >= radius:
                continue
            reach = math.sqrt(radius**2 - across**2)
            start, end = max(p.precision_range_m, along - reach), min(p.view_range_m, along + reach)
            if end <= start + EPS:
                continue
            distance_along = (start + end) / 2
            point = origin[0] + distance_along * c, origin[1] + distance_along * s
            distance, turn = relative(pose, point)
            yaw = normalize(pose[2] + turn)
            if abs(turn) <= MIN_TURN + EPS:
                continue
            if new_view_angle(yaw, self.seen_headings, width) <= MIN_TURN:
                continue
            if any(abs(normalize(yaw - angle)) <= half_view / 2 for angle in active):
                continue
            cost = TRAVEL_COST_WEIGHT * max(0., distance - p.precision_range_m) / p.precision_range_m
            cost += TURN_COST_WEIGHT * abs(turn) / math.pi
            options.append((cost, -item["observed_s"], yaw, key))
        if not options:
            return None
        cost, _, yaw, key = min(options)
        self.deferred_probe_pose = pose[:2]
        action = sim.turn(yaw)
        entry = {"frame_index": self.latest_frame["index"], "pose": pose, "source": "deferred_bearing",
                 "yaw_rad": yaw, "cost": cost, "origin_xy_m": key[0], "bearing_map_rad": key[1],
                 "reason": action["reason"]}
        self.approaches.append(entry)
        if action["status"] != "success":
            return None
        frame = self.capture()
        entry["observation_index"] = frame["index"]
        ray = self.deferred_bearings.get(key)
        confirmed = ray is not None and any(
            (point := intersect_bearings(ray, item, p.precision_range_m, p.view_range_m)) is not None
            and math.dist(pose[:2], point) <= radius + EPS
            for item in frame["detections"] if item["quality"] == "bearing"
        )
        entry["confirmed"] = confirmed or bool(self.local_points())
        if not entry["confirmed"]:
            self.deferred_misses.add(key)
            if sim.turn(pose[2])["status"] == "success":
                self.capture()
                return None
        return frame

    def observe_nearby_bearings(self, frame):
        """Use recent crossings to choose one short observation move."""
        sim, p = self.sim, self.sim.regions
        pose = sim.pose
        if pose[:2] != self.observation_origin:
            self.observation_origin = pose[:2]
            self.observation_attempts.clear()
        current = [item for item in frame["detections"] if item["quality"] == "bearing"]
        options = []
        for origin, rays in self.recent_bearings:
            if math.dist(origin, pose[:2]) < BEARING_STEP_M - EPS:
                continue
            for ray in rays:
                for other in current:
                    point = intersect_bearings(ray, other, p.precision_range_m, p.view_range_m)
                    if point is None:
                        continue
                    distance, turn = relative(pose, point)
                    advance = distance - p.precision_range_m + .10
                    if advance > NEAR_OBSERVATION_REACH:
                        continue
                    yaw = normalize(pose[2] + turn)
                    key = round(yaw, 2)
                    if key in self.observation_attempts:
                        continue
                    cost = TRAVEL_COST_WEIGHT * advance / p.precision_range_m + TURN_COST_WEIGHT * abs(turn) / math.pi
                    options.append((cost, advance, yaw, key))
        if not options:
            return frame
        cost, advance, yaw, key = min(options)
        self.observation_attempts.add(key)
        entry = {"frame_index": frame["index"], "pose": pose, "source": "nearby_bearings", "yaw_rad": yaw,
                 "distance_m": advance, "cost": cost}
        self.approaches.append(entry)
        if abs(normalize(yaw - pose[2])) > MIN_TURN + EPS:
            action = sim.turn(yaw)
            if action["status"] != "success":
                entry["reason"] = action["reason"]
                return frame
            frame = self.capture()
        if not self.local_points() and any(
            item["quality"] == "bearing" and abs(normalize(item["bearing_map_rad"] - sim.pose[2])) <= MIN_TURN + EPS
            for item in frame["detections"]
        ):
            action = sim.forward(min(advance, BEARING_STEP_M))
            entry["reason"] = action["reason"]
            if action["status"] == "success":
                entry["commanded_m"] = min(advance, BEARING_STEP_M)
                frame = self.capture()
        return frame

    def pickup(self, frame):
        sim = self.sim
        points = self.local_points()
        if not points:
            return False, "no_precise_batch"
        attempt = (sim.pose, tuple(sorted(points)))
        if self.last_pickup_attempt and self.last_pickup_attempt[0] == attempt:
            return self.last_pickup_attempt[1]
        target_positions = dict(enumerate(points, 1))
        batch = [{"id": key, "x": x, "y": y} for key, (x, y) in target_positions.items()]
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
                action = sim.execute_pickups(response.goals, output["pickup_stops"], entry["batch_ids"],
                                             frame_index=frame["index"], target_positions=target_positions)
                entry.update(status=action["status"], reason=action["reason"])
                if action["status"] == "success":
                    # Reobserve the previous search direction from the new pose.
                    if self.resume_heading is None and any(item["quality"] == "bearing" for item in frame["known"]):
                        self.resume_heading = frame["pose"][2]
                    picked = [(target["x"], target["y"]) for target in batch]
                    for origin, rays in self.recent_bearings:
                        rays[:] = [ray for ray in rays if not any(
                            abs(relative((*origin, ray["bearing_map_rad"]), point)[1]) <= MIN_TURN + EPS
                            for point in picked)]
                    for key, hint in list(self.memory.items()):
                        if hint["quality"] == "bearing":
                            self.defer_bearing(hint)
                            del self.memory[key]
                        elif (xy := next((xy for xy in picked if same_position(hint["xy_m"], xy)), None)) is not None:
                            del self.memory[key]
                            picked.remove(xy)
                    for key in list(self.deferred_bearings):
                        origin, bearing = key
                        if any(distance > sim.regions.precision_range_m and abs(offset) <= MIN_TURN + EPS
                               for distance, offset in (relative((*origin, bearing), (p["x"], p["y"])) for p in batch)):
                            del self.deferred_bearings[key]
                    # Removing a target may reveal an occluded target from
                    # the same position.
                    self.seen_headings.clear()
        self.plans.append(entry)
        result = entry["status"] == "success", entry["reason"] or entry["status"]
        self.last_pickup_attempt = attempt, result
        return result

    def evaluate(self, frame, tried):
        sim, p = self.sim, self.sim.regions
        success, pickup_reason = self.pickup(frame)
        if success:
            return success, pickup_reason
        yaw, self.resume_heading = self.resume_heading, None
        if (yaw is not None and new_view_angle(yaw, self.seen_headings, math.radians(p.view_angle_deg)) > MIN_TURN + EPS
                and sim.turn(yaw)["status"] == "success"):
            frame = self.capture()
            success, pickup_reason = self.pickup(frame)
            if success:
                return success, pickup_reason
        frame = self.observe_nearby_bearings(frame)
        success, pickup_reason = self.pickup(frame)
        if success:
            return success, pickup_reason
        hints = self.memory.copy()
        points = {key: item["xy_m"] for key, item in hints.items() if item["quality"] == "precise"}
        bearings = {key: item["bearing_map_rad"] for key, item in hints.items()
                    if item["quality"] == "bearing" and item["origin_xy_m"] == sim.pose[:2]}
        ranked = ranked_headings(sim.pose, points, math.radians(p.pickup_angle_deg), p.pickup_inner_radius_m,
                                 bearings, precision_range=p.precision_range_m)
        decision = {"frame_index": frame["index"], "candidates": ranked, "selected": None, "rejected": [],
                    "rule": "coverage_minus_approach_and_turn_cost"}
        self.decisions.append(decision)
        for candidate in ranked:
            start_xy = sim.pose[:2]
            marker = (round(candidate["yaw"], 6), tuple(round(v, 6) for v in start_xy))
            if marker in tried:
                continue
            tried.add(marker)
            reference = hints[candidate["reference_id"]]
            if abs(normalize(candidate["yaw"] - sim.pose[2])) > MIN_TURN + EPS:
                action = sim.turn(candidate["yaw"])
                if action["status"] != "success":
                    decision["rejected"].append({"yaw": candidate["yaw"], "reason": action["reason"]})
                    continue
                frame = self.capture()
            decision["selected"] = candidate
            success, reason = self.pickup(frame)
            if success:
                return success, reason
            frame = self.approach(reference, frame)
            success, reason = self.pickup(frame)
            if success or sim.pose[:2] != start_xy:
                return success, reason
            decision["rejected"].append({"yaw": candidate["yaw"], "reason": reason})
        rejected = decision["rejected"]
        return False, rejected[-1]["reason"] if rejected else pickup_reason

    def scan(self):
        sim = self.sim
        width = math.radians(sim.regions.view_angle_deg)
        sectors = math.ceil(360 / sim.regions.view_angle_deg)
        options = {normalize(sim.pose[2] + i * math.tau / sectors) for i in range(sectors)}
        start_xy = sim.pose[:2]
        frame = self.recheck_deferred()
        if frame is not None:
            yield frame
            if sim.pose[:2] != start_xy:
                return
        tried = set()
        while True:
            x, y, heading = sim.pose
            bearings = []
            for item in self.memory.values():
                if item["quality"] == "precise":
                    px, py = item["xy_m"]
                    bearings.append(math.atan2(py - y, px - x))
                else:
                    bearings.append(item["bearing_map_rad"])
            candidates = []
            for yaw in (options | set(bearings)) - tried:
                turn = abs(normalize(yaw - heading))
                if turn <= MIN_TURN + EPS:
                    continue
                gain = new_view_angle(yaw, self.seen_headings, width)
                if gain > MIN_TURN + EPS:
                    count = sum(abs(normalize(yaw - b)) <= width / 2 + EPS for b in bearings)
                    candidates.append((-count, -round(gain, 9), turn, yaw))
            if not candidates:
                return
            yaw = min(candidates)[-1]
            tried.add(yaw)
            if sim.turn(yaw)["status"] == "success":
                frame = self.capture()
                yield frame
                if frame["pose"][:2] != sim.pose[:2]:
                    return
