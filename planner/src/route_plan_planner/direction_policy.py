"""Directional pickup strategy driven by stopped observations and action feedback."""

from collections import deque
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from uuid import uuid4

from route_plan_sim.scene import EPS, normalize

from .api import PickupStop, PlanningRequest, Target, plan
from .cluster_memory import ClusterMemory, CoverageCluster, coordinates, match_positions, relative
from .motion import MIN_ROTATION as MIN_TURN, MIN_TRANSLATION as MIN_DISTANCE, Pose


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


@dataclass(frozen=True)
class CoverageConfig:
    view_range_m: float = 5.
    view_angle_deg: float = 90.
    pickup_inner_radius_m: float = .75
    pickup_angle_deg: float = 60.
    approach_tolerance_m: float = .05
    precision_range_m: float = 1.4
    auto_fence_radius_m: float = 1.4
    maximum_rounds: int = 80
    position_match_tolerance_m: float = EPS
    probe_initial_heading: bool = False
    cluster_radius_m: float | None = None
    coarse_match_tolerance_m: float = .25

    def __post_init__(self):
        distances = (self.view_range_m, self.pickup_inner_radius_m, self.precision_range_m, self.auto_fence_radius_m)
        if any(not math.isfinite(value) or value <= 0 for value in distances):
            raise ValueError("Coverage distances must be finite and positive")
        if not (0 < self.view_angle_deg <= 360 and 0 < self.pickup_angle_deg <= 360
                and 0 <= self.approach_tolerance_m < math.inf
                and 0 <= self.position_match_tolerance_m < math.inf):
            raise ValueError("Invalid coverage angles or approach tolerance")
        if type(self.maximum_rounds) is not int or self.maximum_rounds < 1:
            raise ValueError("maximum_rounds must be a positive integer")
        if self.cluster_radius_m is not None and not 0 < self.cluster_radius_m < math.inf:
            raise ValueError("cluster_radius_m must be finite and positive")
        if not 0 < self.coarse_match_tolerance_m < math.inf:
            raise ValueError("coarse_match_tolerance_m must be finite and positive")


@dataclass(frozen=True)
class CoverageAction:
    action_id: str
    kind: str
    frame_id: str
    start_pose: Pose | None
    goals: tuple[Pose, ...] = ()
    pickup_stops: tuple[PickupStop, ...] = ()
    targets: tuple[Target, ...] = ()
    distance_m: float | None = None
    auto_fence_radius_m: float | None = None


class _RefreshObservation(Exception):
    """The actual robot pose changed during a failed action."""


class _StopCoverage(Exception):
    """The session should return control to the caller."""


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
    """Keep one strategy instance per robot task. Call serially from the event loop."""

    def __init__(self, config=None, *, request_options=None, planner=plan, use_deferred_bearings=False):
        self.config = config if config is not None else CoverageConfig()
        self.request_options = deepcopy(dict(request_options or {}))
        self.frame_id = self.request_options.get("frame_id", "map")
        self.plan = planner
        self.pose = None
        self.stop_reason = None
        self.pending_action = None
        self._session_id = uuid4().hex
        self._action_number = 0
        self._result_time = 0.
        self._next_detection_id = 1
        self._run = self._run_policy() if self.config.cluster_radius_m is None else self._run_clusters()
        self._observation_ids = set()
        self.use_deferred_bearings = use_deferred_bearings
        self._memory = {}
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
        self.active_cluster = None
        self._clusters = (ClusterMemory(cluster_radius_m=self.config.cluster_radius_m,
                                       coarse_match_tolerance_m=self.config.coarse_match_tolerance_m)
                          if self.config.cluster_radius_m is not None else None)
        self._selected_clusters: deque[CoverageCluster] = deque()

    def is_next_cluster(self) -> bool:
        """Check whether a newly selected cluster is available."""
        return bool(self._selected_clusters)

    def next_cluster(self) -> CoverageCluster | None:
        """Consume the oldest unread selection. Robot actions advance separately."""
        return self._selected_clusters.popleft() if self._selected_clusters else None

    def next_action(self) -> CoverageAction | None:
        """Return the outstanding action or the first action. None means stopped."""
        if self.pending_action is None and self.stop_reason is None:
            self._advance(None)
        return self.pending_action

    def submit_observation(self, action_id, *, pose, time_s, observation,
                           supplemental_detections=None) -> bool:
        """Complete an observe action with detection batches from this stop."""
        action = self.pending_action
        if action is None or action.action_id != action_id or action.kind != "observe":
            return False
        if not isinstance(observation, Mapping):
            raise ValueError("An observe result requires an observation")
        if supplemental_detections is not None:
            observation = {**observation, "supplemental_detections": supplemental_detections}
        return self.submit_result(action_id, status="success", pose=pose, time_s=time_s, observation=observation)

    def submit_result(self, action_id, *, status, pose, time_s, observation=None, picked_ids=(), reason=None) -> bool:
        """Advance on a matching, settled result. Ignore late or duplicate action IDs."""
        action = self.pending_action
        if action is None or action.action_id != action_id:
            return False
        if status not in ("success", "blocked", "failed", "partial", "timeout", "cancelled"):
            raise ValueError("Unknown action status")
        if status == "partial" and action.kind != "pickup":
            raise ValueError("Only a pickup batch can be partially successful")
        measured = coordinates(pose, 3, "pose")
        if type(time_s) not in (int, float) or not math.isfinite(time_s) or time_s < self._result_time:
            raise ValueError("Feedback time must be finite and nondecreasing")
        picked = tuple(picked_ids)
        allowed = {target for stop in action.pickup_stops for target in stop.target_ids}
        if (any(type(key) is not int or key not in allowed for key in picked)
                or len(set(picked)) != len(picked)):
            raise ValueError("picked_ids must be unique request IDs from the pickup action")
        if action.kind == "pickup" and status == "success" and set(picked) != allowed:
            raise ValueError("A successful pickup must confirm all requested IDs")
        if status == "partial" and (not picked or set(picked) == allowed):
            raise ValueError("A partial pickup must confirm only the successful subset")
        frame = None
        if action.kind == "observe" and status == "success":
            frame = self._observation(observation, measured, time_s)
        elif observation is not None:
            raise ValueError("An observation is accepted only for a successful observe action")
        self.pose = measured
        self._result_time = time_s
        self._advance({"status": status, "reason": reason, "frame": frame, "picked_ids": picked})
        return True

    def _observation(self, observation, pose, time_s):
        if not isinstance(observation, Mapping):
            raise ValueError("An observe result requires an observation")
        frame_id = observation.get("frame_id", self.frame_id)
        observation_id = observation.get("observation_id")
        if frame_id != self.frame_id:
            raise ValueError("Observation frame_id differs from the strategy frame")
        if (type(observation_id) not in (int, str) or observation_id == ""
                or observation_id in self._observation_ids):
            raise ValueError("observation_id must be unique for this task")
        captured_at = observation.get("time_s", time_s)
        if (type(captured_at) not in (int, float) or not math.isfinite(captured_at)
                or not self._result_time <= captured_at <= time_s):
            raise ValueError("Observation must have been captured after the previous result")
        supplements = observation.get("supplemental_detections", ())
        if not isinstance(supplements, (list, tuple)):
            raise ValueError("supplemental_detections must contain detection lists")
        if supplements and self.config.cluster_radius_m is None:
            raise ValueError("Supplemental detections require cluster mode")
        batches = []
        next_id = self._next_detection_id
        for batch in (observation.get("detections"), *supplements):
            parsed = self._parse_detections(batch, pose, next_id)
            batches.append(parsed)
            next_id += len(parsed)
        index = len(self._observation_ids)
        self._observation_ids.add(observation_id)
        self._next_detection_id = next_id
        frame = {"index": index, "observation_id": observation_id, "time_s": captured_at, "pose": pose,
                 "frame_id": frame_id, "view_range_m": self.config.view_range_m,
                 "view_angle_deg": self.config.view_angle_deg, "detections": batches[0]}
        if supplements:
            frame["supplemental_detections"] = batches[1:]
        return frame

    def _parse_detections(self, batch, pose, first_id):
        if not isinstance(batch, (list, tuple)):
            raise ValueError("detections must be a list")
        detections = []
        for item in batch:
            if not isinstance(item, Mapping):
                raise ValueError("Each detection must be a mapping")
            quality = item.get("quality")
            detection = {"id": first_id + len(detections), "quality": quality}
            if quality == "precise":
                detection["xy_m"] = coordinates(item.get("xy_m"), 2, "xy_m")
                if math.dist(pose[:2], detection["xy_m"]) > self.config.precision_range_m + EPS:
                    raise ValueError("A precise detection must be within precision_range_m")
            elif quality == "coarse" and self.config.cluster_radius_m is not None:
                detection["xy_m"] = coordinates(item.get("xy_m"), 2, "xy_m")
            elif quality == "bearing" and self.config.cluster_radius_m is None:
                detection["origin_xy_m"] = coordinates(item.get("origin_xy_m"), 2, "origin_xy_m")
                bearing, = coordinates([item.get("bearing_map_rad")], 1, "bearing_map_rad")
                detection["bearing_map_rad"] = bearing
            else:
                raise ValueError("Detection quality must be precise or bearing")
            detections.append(detection)
        return detections

    def _advance(self, feedback):
        self.pending_action = None
        try:
            self.pending_action = next(self._run) if feedback is None else self._run.send(feedback)
        except StopIteration as stopped:
            self.stop_reason = stopped.value
        except _StopCoverage as stopped:
            self.stop_reason = str(stopped)

    def _action(self, kind, **options):
        self._action_number += 1
        return (yield CoverageAction(f"{self._session_id}:{self._action_number}", kind, self.frame_id,
                                     self.pose, **options))

    def _motion(self, kind, goal, **options):
        before = self.pose
        result = yield from self._action(kind, goals=(goal,), **options)
        if result["status"] in ("timeout", "cancelled"):
            raise _StopCoverage(result["status"])
        if result["status"] != "success" and self.pose != before:
            raise _RefreshObservation
        return result

    def _turn(self, yaw):
        return (yield from self._motion("turn", (*self.pose[:2], yaw)))

    def _forward(self, distance):
        x, y, yaw = self.pose
        goal = x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw
        return (yield from self._motion("forward", goal, distance_m=distance))

    def _same_position(self, left, right):
        return math.dist(left, right) <= self.config.position_match_tolerance_m

    def _read_frame(self):
        result = yield from self._action("observe")
        if result["status"] != "success":
            raise _StopCoverage(result["status"])
        frame = result["frame"]
        x, y, heading = frame["pose"]
        same_station = (math.dist((x, y), self.sweep_origin) <= MIN_DISTANCE
                        if self.sweep_origin is not None and self.config.cluster_radius_m is not None
                        else (x, y) == self.sweep_origin)
        if not same_station:
            self.sweep_origin = (x, y)
            self.seen_headings.clear()
        if heading not in self.seen_headings:
            self.seen_headings.append(heading)
        return frame

    @property
    def memory(self):
        if self._clusters is None:
            return self._memory
        coarse = {hint.id: {**asdict(hint), "quality": "coarse"} for hint in self._clusters.hints}
        return {**self._memory, **coarse}

    def _capture_clusters(self):
        frame = yield from self._read_frame()
        for batch in (frame["detections"], *frame.get("supplemental_detections", ())):
            self._merge_search_observations(batch, frame["index"])
        if not self.local_points():
            self._record_checked_hints(frame)
        frame["known"] = list(self.memory.values())
        frame["deferred"] = []
        frame["clusters"] = self._clusters.summary(self.active_cluster)
        self.latest_frame = frame
        return frame

    def _merge_search_observations(self, detections, frame_index):
        precise = [item for item in detections if item["quality"] == "precise"]
        positions = [item["xy_m"] for item in precise]
        measured = {key: item["xy_m"] for key, item in self._memory.items()}
        matches = match_positions(positions, measured, self.config.position_match_tolerance_m)
        for index, item in enumerate(precise):
            key = matches.get(index, item["id"])
            self._memory[key] = {**item, "id": key, "frame_index": frame_index}
        self._clusters.remember((item["xy_m"] for item in detections if item["quality"] == "coarse"),
                                frame_index=frame_index)

    def _select_cluster_target(self, frame):
        hints = self._clusters.pending_hints(self.active_cluster)
        if not hints:
            candidates = [cluster for cluster in self._clusters.summary() if cluster["pending"]]
            selected = self._clusters.next_cluster(self.pose)
            if selected is None:
                self.active_cluster = None
                return None
            self.active_cluster = selected.cluster_id
            self._selected_clusters.append(selected)
            candidates.sort(key=lambda cluster: cluster["id"] != self.active_cluster)
            self.decisions.append({"frame_index": frame["index"], "rule": "cluster_density",
                                   "selected": self.active_cluster, "candidates": candidates})
            hints = selected.hints
        return min(hints, key=lambda hint: (
            relative(self.pose, hint.xy_m)[0] + .25 * abs(relative(self.pose, hint.xy_m)[1]), hint.id
        )).id

    def _visit_cluster_target(self, key, frame):
        # Track the same spatial hypothesis through the fresh observations
        # after each short motion. Only the precise memory goes to _pickup().
        start_distance = math.dist(self.pose[:2], self._clusters.hint(key).xy_m)
        for _ in range(math.ceil(self.config.view_range_m / BEARING_STEP_M) + 2):
            hint = self._clusters.hint(key)
            distance, offset = relative(self.pose, hint.xy_m)
            entry = {"frame_index": frame["index"], "pose": self.pose, "reference_id": key,
                     "cluster_id": self.active_cluster, "source": "cluster",
                     "yaw_rad": normalize(self.pose[2] + offset), "reason": None}
            self.approaches.append(entry)
            if self._covered_cluster_miss(hint, frame):
                entry["reason"] = "precision_recheck_exhausted"
                break
            if abs(offset) > MIN_TURN + EPS:
                result = yield from self._turn(entry["yaw_rad"])
                if result["status"] == "success":
                    frame = yield from self._capture_clusters()
                    if not self._clusters.is_pending(key) or self.local_points():
                        return frame
                else:
                    entry["reason"] = result["reason"]
                    break
            hint = self._clusters.hint(key)
            distance, offset = relative(self.pose, hint.xy_m)
            if abs(offset) > min(MAX_SIDE_PROBE_TURN, math.radians(self.config.view_angle_deg / 2)):
                continue
            if self._covered_cluster_miss(hint, frame):
                entry["reason"] = "precision_recheck_exhausted"
                break
            advance = min(BEARING_STEP_M, max(.15, distance - self.config.precision_range_m + .15))
            entry["distance_m"] = advance
            for step in (advance, advance / 2):
                result = yield from self._forward(step)
                if result["status"] == "success":
                    entry["commanded_m"] = step
                    frame = yield from self._capture_clusters()
                    break
                entry["reason"] = result["reason"]
            else:
                break
            if not self._clusters.is_pending(key) or self.local_points():
                return frame
        else:
            # A remembered cluster may now be farther away than the camera
            # range. Give other rounds a chance to continue steady progress.
            if math.dist(self.pose[:2], self._clusters.hint(key).xy_m) <= start_distance - BEARING_STEP_M:
                return frame
            entry["reason"] = "cluster_approach_limit"
        self._clusters.mark_attempted(key)
        return frame

    def _record_checked_hints(self, frame):
        for hint in self._clusters.hints:
            if self._covered_cluster_miss(hint, frame):
                self._clusters.mark_attempted(hint.id, missed=True)

    def _covered_cluster_miss(self, hint, frame):
        distance, offset = relative(frame["pose"], hint.xy_m)
        return (hint.frame_index != frame["index"] and distance <= self.config.precision_range_m
                and abs(offset) <= math.radians(.4 * frame["view_angle_deg"]))

    def _run_clusters(self):
        frame = yield from self._capture_clusters()
        width = math.radians(self.config.view_angle_deg)
        sectors = math.ceil(360 / self.config.view_angle_deg)
        scanned = set()
        rounds = 0
        # Leave room for the final views even if the last pickup used the
        # search budget. Also bound consecutive scans if the stop pose drifts.
        for _ in range(self.config.maximum_rounds + sectors + 1):
            try:
                if rounds >= self.config.maximum_rounds and (
                    self.local_points() or self._clusters.pending_hints(self.active_cluster)
                    or self._clusters.is_next_cluster()
                ):
                    return "round_limit"
                success, _ = yield from self._pickup(frame)
                if success:
                    rounds += 1
                    scanned.clear()
                    frame = yield from self._capture_clusters()
                    continue
                key = self._select_cluster_target(frame)
                if key is not None:
                    origin = self.sweep_origin
                    frame = yield from self._visit_cluster_target(key, frame)
                    rounds += 1
                    if self.sweep_origin != origin:
                        scanned.clear()
                    continue
                headings = [normalize(self.seen_headings[0] + i * math.tau / sectors) for i in range(sectors)]
                uncovered = [yaw for yaw in headings
                             if new_view_angle(yaw, self.seen_headings, width) > MIN_TURN + EPS]
                options = [yaw for yaw in uncovered if yaw not in scanned]
                if not options:
                    if uncovered:
                        return "incomplete_final_scan"
                    if self._memory:
                        return "unresolved_precise_targets"
                    return ("unresolved_cluster_hints" if self._clusters.has_unresolved_hints()
                            else "no_observed_targets")
                yaw = min(options, key=lambda yaw: abs(normalize(yaw - self.pose[2])))
                scanned.add(yaw)
                self.decisions.append({"frame_index": frame["index"], "rule": "cluster_final_scan", "yaw": yaw})
                if (yield from self._turn(yaw))["status"] == "success":
                    frame = yield from self._capture_clusters()
            except _RefreshObservation:
                rounds += 1
                scanned.clear()
                frame = yield from self._capture_clusters()
        return "round_limit"

    def _capture(self):
        frame = yield from self._read_frame()
        x, y, heading = frame["pose"]
        half_view = math.radians(frame["view_angle_deg"] / 2)
        bearings = [item for item in frame["detections"] if item["quality"] == "bearing"]
        current_keys = {self.bearing_key(ray) for ray in bearings}
        for key, item in list(self._memory.items()):
            if item["quality"] == "bearing" and (
                item["station_xy_m"] != (x, y)
                or abs(normalize(item["bearing_map_rad"] - heading)) <= half_view + EPS
            ):
                if self.bearing_key(item) not in current_keys:
                    self.defer_bearing(item)
                del self._memory[key]
        available = {key for key, item in self._memory.items() if item["quality"] == "precise"}
        for item in frame["detections"]:
            key = item["id"]
            if item["quality"] == "precise":
                key = min(
                    (key for key in self._memory if key in available
                     and self._same_position(self._memory[key]["xy_m"], item["xy_m"])),
                    key=lambda key: math.dist(self._memory[key]["xy_m"], item["xy_m"]), default=None,
                )
                if key is None:
                    key = self.next_point_id
                    self.next_point_id -= 1
                available.discard(key)
            self._memory[key] = {**item, "id": key}
            if item["quality"] == "bearing":
                self._memory[key]["observed_s"] = frame["time_s"]
                self._memory[key]["station_xy_m"] = (x, y)
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
        frame["known"] = list(self._memory.values())
        frame["deferred"] = list(self.deferred_bearings.values())
        self.latest_frame = frame
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

    def _approach(self, hint, frame):
        p = self.config
        reference = hint["id"]
        missed_bearings = []
        for _ in range(20):
            if hint["quality"] == "precise":
                item = next((item for item in frame["detections"]
                             if item["quality"] == "precise" and self._same_position(item["xy_m"], hint["xy_m"])), None)
            else:
                item = min((item for item in frame["detections"] if item["quality"] == "bearing"
                            and abs(normalize(item["bearing_map_rad"] - self.pose[2])) <= MIN_TURN + EPS),
                           key=lambda item: abs(normalize(item["bearing_map_rad"] - self.pose[2])), default=None)
            entry = {"reference_id": reference, "pose": self.pose, "frame_index": frame["index"]}
            if item is None:
                distance, reason = None, "reference_not_observed"
            elif item["quality"] == "bearing":
                entry["source"] = "bearing"
                distance, reason = BEARING_STEP_M, None
            else:
                radial, bearing = relative(self.pose, item["xy_m"])
                distance, reason = approach_distance(radial * math.cos(bearing), radial * math.sin(bearing),
                                                     p.pickup_inner_radius_m, p.approach_tolerance_m)
            entry.update(distance_m=distance, reason=reason)
            self.approaches.append(entry)
            if distance is None or distance == 0:
                return frame
            entry["commanded_m"] = distance
            action = yield from self._forward(distance)
            if action["status"] != "success":
                entry["reason"] = action["reason"]
                return frame
            previous, frame = frame, (yield from self._capture())
            if self.local_points():
                return frame
            if item["quality"] == "bearing":
                bearings = [item for item in previous["detections"] if item["quality"] == "bearing"]
                frame, missed_bearings = (yield from self._probe_side_bearings(bearings, missed_bearings, frame))
                if frame["pose"][2] != previous["pose"][2]:
                    return frame
        self.approaches.append({"reference_id": reference, "reason": "approach_limit", "pose": self.pose})
        return frame

    def _probe_side_bearings(self, bearings, previous_misses, frame):
        # The projected rays choose where to look. Only capture() can produce a
        # measured point for pickup, and turn() checks the entire swept body.
        p = self.config
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
        action = yield from self._turn(yaw)
        self.approaches.append({"pose": frame["pose"], "frame_index": frame["index"], "source": "bearing_leaving_view",
                                "yaw_rad": yaw, "reason": action["reason"]})
        if action["status"] != "success":
            return frame, missed
        frame = (yield from self._capture())
        if self.local_points():
            if self.resume_heading is None:
                self.resume_heading = heading
            return frame, []
        if (yield from self._turn(heading))["status"] == "success":
            frame = (yield from self._capture())
        return frame, missed

    def local_points(self):
        radius = self.config.precision_range_m
        return [item["xy_m"] for item in self._memory.values()
                if item["quality"] == "precise" and math.dist(self.pose[:2], item["xy_m"]) <= radius + EPS]

    def _recheck_deferred(self):
        """Look toward old rays from this pose, and let fresh detections drive any move."""
        p = self.config
        pose = self.pose
        if not self.deferred_bearings or self.deferred_probe_pose == pose[:2]:
            return None
        radius = p.precision_range_m + DEFERRED_RECHECK_DISTANCE
        half_view = math.radians(p.view_angle_deg / 2)
        width = 2 * half_view
        active = [item["bearing_map_rad"] for item in self._memory.values() if item["quality"] == "bearing"]
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
        action = yield from self._turn(yaw)
        entry = {"frame_index": self.latest_frame["index"], "pose": pose, "source": "deferred_bearing",
                 "yaw_rad": yaw, "cost": cost, "origin_xy_m": key[0], "bearing_map_rad": key[1],
                 "reason": action["reason"]}
        self.approaches.append(entry)
        if action["status"] != "success":
            return None
        frame = (yield from self._capture())
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
            if (yield from self._turn(pose[2]))["status"] == "success":
                (yield from self._capture())
                return None
        return frame

    def _observe_nearby_bearings(self, frame):
        """Use recent crossings to choose one short observation move."""
        p = self.config
        pose = self.pose
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
            action = yield from self._turn(yaw)
            if action["status"] != "success":
                entry["reason"] = action["reason"]
                return frame
            frame = (yield from self._capture())
        if not self.local_points() and any(
            item["quality"] == "bearing" and abs(normalize(item["bearing_map_rad"] - self.pose[2])) <= MIN_TURN + EPS
            for item in frame["detections"]
        ):
            action = yield from self._forward(min(advance, BEARING_STEP_M))
            entry["reason"] = action["reason"]
            if action["status"] == "success":
                entry["commanded_m"] = min(advance, BEARING_STEP_M)
                frame = (yield from self._capture())
        return frame

    def _pickup(self, frame):
        points = self.local_points()
        if not points:
            return False, "no_precise_batch"
        attempt = (self.pose, tuple(sorted(points)))
        if self.last_pickup_attempt and self.last_pickup_attempt[0] == attempt:
            return self.last_pickup_attempt[1]
        target_positions = dict(enumerate(points, 1))
        batch = [{"id": key, "x": x, "y": y} for key, (x, y) in target_positions.items()]
        payload = {"use_ompl": False, "timeout": 2., "auto_fence_radius_m": self.config.auto_fence_radius_m,
                   **self.request_options,
                   "pose": self.pose, "targets": batch, "mode": "auto_fence", "method": "stops"}
        entry = {"frame_index": frame["index"], "request": payload,
                 "batch_ids": list(target_positions)}
        progress = False
        try:
            request = PlanningRequest.from_dict(payload)
            entry["request"] = request.to_dict()
            response = self.plan(request)
        except ValueError as error:
            entry.update(status="rejected", reason=str(error))
        else:
            output = response.to_dict()
            # Elapsed wall time is kept for diagnostics, independent of simulated time.
            entry.update(status=response.status, reason=response.reason, response=output)
            if response.success:
                action = yield from self._action(
                    "pickup", goals=tuple(response.goals), pickup_stops=tuple(response.pickup_stops),
                    targets=request.targets, auto_fence_radius_m=request.auto_fence_radius_m,
                )
                entry.update(status=action["status"], reason=action["reason"])
                progress = bool(action["picked_ids"])
                if progress:
                    # Reobserve the previous search direction from the new pose.
                    if self.resume_heading is None and any(item["quality"] == "bearing" for item in frame["known"]):
                        self.resume_heading = frame["pose"][2]
                    picked = [target_positions[key] for key in action["picked_ids"]]
                    confirmed = tuple(picked)
                    for _, rays in self.recent_bearings:
                        rays[:] = [ray for ray in rays if not any(
                            abs(relative((*ray["origin_xy_m"], ray["bearing_map_rad"]), point)[1]) <= MIN_TURN + EPS
                            for point in picked)]
                    for key, hint in list(self._memory.items()):
                        if hint["quality"] == "bearing":
                            self.defer_bearing(hint)
                            del self._memory[key]
                        elif hint["quality"] == "precise" and hint["xy_m"] in picked:
                            del self._memory[key]
                            picked.remove(hint["xy_m"])
                    for key in list(self.deferred_bearings):
                        origin, bearing = key
                        if any(distance > self.config.precision_range_m and abs(offset) <= MIN_TURN + EPS
                               for distance, offset in (relative((*origin, bearing), point) for point in confirmed)):
                            del self.deferred_bearings[key]
                    # Removing a target may reveal an occluded target from
                    # the same position.
                    self.seen_headings.clear()
        self.plans.append(entry)
        result = progress, entry["reason"] or entry["status"]
        self.last_pickup_attempt = attempt, result
        if entry["status"] in ("timeout", "cancelled"):
            raise _StopCoverage(entry["status"])
        if not progress and self.pose != frame["pose"]:
            raise _RefreshObservation
        return result

    def _evaluate(self, frame, tried):
        p = self.config
        success, pickup_reason = (yield from self._pickup(frame))
        if success:
            return success, pickup_reason
        yaw, self.resume_heading = self.resume_heading, None
        if (yaw is not None and new_view_angle(yaw, self.seen_headings, math.radians(p.view_angle_deg)) > MIN_TURN + EPS
                and (yield from self._turn(yaw))["status"] == "success"):
            frame = (yield from self._capture())
            success, pickup_reason = (yield from self._pickup(frame))
            if success:
                return success, pickup_reason
        frame = (yield from self._observe_nearby_bearings(frame))
        success, pickup_reason = (yield from self._pickup(frame))
        if success:
            return success, pickup_reason
        hints = self._memory.copy()
        points = {key: item["xy_m"] for key, item in hints.items() if item["quality"] == "precise"}
        bearings = {key: item["bearing_map_rad"] for key, item in hints.items()
                    if item["quality"] == "bearing" and item["station_xy_m"] == self.pose[:2]}
        ranked = ranked_headings(self.pose, points, math.radians(p.pickup_angle_deg), p.pickup_inner_radius_m,
                                 bearings, precision_range=p.precision_range_m)
        decision = {"frame_index": frame["index"], "candidates": ranked, "selected": None, "rejected": [],
                    "rule": "coverage_minus_approach_and_turn_cost"}
        self.decisions.append(decision)
        for candidate in ranked:
            start_xy = self.pose[:2]
            marker = (round(candidate["yaw"], 6), tuple(round(v, 6) for v in start_xy))
            if marker in tried:
                continue
            tried.add(marker)
            reference = hints[candidate["reference_id"]]
            if abs(normalize(candidate["yaw"] - self.pose[2])) > MIN_TURN + EPS:
                action = yield from self._turn(candidate["yaw"])
                if action["status"] != "success":
                    decision["rejected"].append({"yaw": candidate["yaw"], "reason": action["reason"]})
                    continue
                frame = (yield from self._capture())
            decision["selected"] = candidate
            success, reason = (yield from self._pickup(frame))
            if success:
                return success, reason
            frame = (yield from self._approach(reference, frame))
            success, reason = (yield from self._pickup(frame))
            if success or self.pose[:2] != start_xy:
                return success, reason
            decision["rejected"].append({"yaw": candidate["yaw"], "reason": reason})
        rejected = decision["rejected"]
        return False, rejected[-1]["reason"] if rejected else pickup_reason

    def _run_policy(self):
        frame = yield from self._capture()
        if self.config.probe_initial_heading and frame["detections"]:
            try:
                ok, _ = yield from self._pickup(frame)
                if not ok:
                    frame = yield from self._approach(frame["detections"][0], frame)
                    yield from self._pickup(frame)
            except _RefreshObservation:
                pass
            frame = yield from self._capture()
        tried, last_failure = set(), None
        for _ in range(self.config.maximum_rounds):
            start_xy = self.pose[:2]
            try:
                ok, reason = yield from self._evaluate(frame, tried)
                if not ok and reason != "no_safe_heading":
                    last_failure = reason
                if not ok and self.pose[:2] == start_xy:
                    ok, reason = yield from self._scan(tried)
                    if not ok and reason != "no_safe_heading":
                        last_failure = reason
                if not ok and self.pose[:2] == start_xy:
                    return (last_failure or "no_safe_heading") if self._memory else "no_observed_targets"
            except _RefreshObservation:
                ok = False
            if ok:
                tried.clear()
                last_failure = None
            frame = yield from self._capture()
        return "round_limit"

    def _scan(self, tried):
        width = math.radians(self.config.view_angle_deg)
        sectors = math.ceil(360 / self.config.view_angle_deg)
        options = {normalize(self.pose[2] + i * math.tau / sectors) for i in range(sectors)}
        start_xy = self.pose[:2]
        frame = (yield from self._recheck_deferred())
        reason = "no_safe_heading"
        if frame is not None:
            ok, reason = yield from self._evaluate(frame, tried)
            if ok or self.pose[:2] != start_xy:
                return ok, reason
        scanned = set()
        while True:
            x, y, heading = self.pose
            bearings = []
            for item in self._memory.values():
                if item["quality"] == "precise":
                    px, py = item["xy_m"]
                    bearings.append(math.atan2(py - y, px - x))
                else:
                    bearings.append(item["bearing_map_rad"])
            candidates = []
            for yaw in (options | set(bearings)) - scanned:
                turn = abs(normalize(yaw - heading))
                if turn <= MIN_TURN + EPS:
                    continue
                gain = new_view_angle(yaw, self.seen_headings, width)
                if gain > MIN_TURN + EPS:
                    count = sum(abs(normalize(yaw - b)) <= width / 2 + EPS for b in bearings)
                    candidates.append((-count, -round(gain, 9), turn, yaw))
            if not candidates:
                return False, reason
            yaw = min(candidates)[-1]
            scanned.add(yaw)
            if (yield from self._turn(yaw))["status"] == "success":
                frame = (yield from self._capture())
                ok, latest = yield from self._evaluate(frame, tried)
                if latest != "no_safe_heading":
                    reason = latest
                if ok or self.pose[:2] != start_xy:
                    return ok, reason
