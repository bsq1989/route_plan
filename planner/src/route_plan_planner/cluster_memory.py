"""Coarse observation memory and selection for a single search task."""

from dataclasses import dataclass
import heapq
import math

from route_plan_sim.boundary import EPS
from route_plan_sim.scene import normalize


def relative(pose, point):
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    offset = normalize(math.atan2(dy, dx) - pose[2])
    return math.hypot(dx, dy), offset


def coordinates(values, size, name):
    if not isinstance(values, (tuple, list)) or len(values) != size or any(
        type(value) not in (int, float) or not math.isfinite(value) for value in values
    ):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return tuple(values)


def match_positions(points, stored, tolerance):
    """Return an association from input indices to stored keys, closest pairs first."""
    pairs = []
    for index, point in enumerate(points):
        for key, previous in stored.items():
            distance = math.dist(point, previous)
            if distance <= tolerance:
                pairs.append((distance, index, key))
    matches = {}
    used = set()
    for _, index, key in sorted(pairs):
        if index not in matches and key not in used:
            matches[index] = key
            used.add(key)
    return matches


@dataclass(frozen=True)
class ClusterHint:
    id: int
    xy_m: tuple[float, float]
    cluster_id: int
    frame_index: int


@dataclass(frozen=True)
class CoverageCluster:
    """A snapshot of a selected cluster and its pending coarse hints."""

    cluster_id: int
    anchor_xy_m: tuple[float, float]
    radius_m: float
    pending_count: int
    hints: tuple[ClusterHint, ...] = ()


class ClusterMemory:
    """Add coarse points and pop clusters explicitly, in one fixed metric frame."""

    def __init__(self, *, cluster_radius_m=1.6, coarse_match_tolerance_m=.25):
        if not 0 < cluster_radius_m < math.inf:
            raise ValueError("cluster_radius_m must be finite and positive")
        if not 0 < coarse_match_tolerance_m < math.inf:
            raise ValueError("coarse_match_tolerance_m must be finite and positive")
        self.cluster_radius_m = cluster_radius_m
        self.coarse_match_tolerance_m = coarse_match_tolerance_m
        self._anchors = {}
        self._hints = {}
        self._attempted = set()
        self._misses = set()
        self._popped = set()
        self._last_frame = -1

    def remember(self, points, *, frame_index=None):
        """Merge one batch of observed [x, y] points and return their hint records."""
        observed = [coordinates(point, 2, "coarse point") for point in points]
        if frame_index is None:
            frame_index = self._last_frame + 1
        if type(frame_index) is not int or frame_index < max(0, self._last_frame):
            raise ValueError("frame_index must be a nondecreasing nonnegative integer")
        stored = {key: hint.xy_m for key, hint in self._hints.items()}
        matches = match_positions(observed, stored, self.coarse_match_tolerance_m)
        remembered = []
        for index, point in enumerate(observed):
            key = matches.get(index)
            if key in self._misses:
                self._misses.discard(key)
                self._attempted.discard(key)
            cluster_id = self._hints[key].cluster_id if key is not None else None
            radius = self.cluster_radius_m + EPS
            if cluster_id is None or math.dist(point, self._anchors[cluster_id]) > radius:
                choices = [(math.dist(point, anchor), cid) for cid, anchor in self._anchors.items()]
                cluster_id = min((c for c in choices if c[0] <= radius), default=(0., None))[1]
            if cluster_id is None:
                cluster_id = len(self._anchors) + 1
                self._anchors[cluster_id] = point
            if key is None:
                key = -len(self._hints) - 1
            hint = ClusterHint(key, point, cluster_id, frame_index)
            self._hints[key] = hint
            remembered.append(hint)
        self._last_frame = frame_index
        return tuple(remembered)

    @property
    def hints(self):
        return tuple(self._hints.values())

    def hint(self, hint_id):
        return self._hints[hint_id]

    def pending_hints(self, cluster_id):
        return tuple(hint for hint in self._hints.values()
                     if hint.cluster_id == cluster_id and self.is_pending(hint.id))

    def is_pending(self, hint_id):
        return hint_id in self._hints and hint_id not in self._attempted

    def mark_attempted(self, hint_id, *, missed=False):
        """Record an attempted hint. A fresh detection reopens a missed hint."""
        if hint_id not in self._hints:
            raise KeyError(hint_id)
        self._attempted.add(hint_id)
        if missed:
            self._misses.add(hint_id)

    def has_unresolved_hints(self):
        return bool(self._hints.keys() - self._misses)

    def summary(self, active_cluster=None):
        groups = {}
        for hint in self._hints.values():
            groups.setdefault(hint.cluster_id, []).append(hint.id)
        return [{"id": cid, "anchor_xy_m": self._anchors[cid], "remaining": len(keys),
                 "pending": sum(self.is_pending(key) for key in keys),
                 "active": active_cluster == cid}
                for cid, keys in groups.items() if cid == active_cluster or cid not in self._popped]

    def is_next_cluster(self):
        return any(self.is_pending(hint.id) and hint.cluster_id not in self._popped
                   for hint in self._hints.values())

    def next_cluster(self, pose) -> CoverageCluster | None:
        """Pop the densest remaining cluster using the supplied measured pose."""
        pose = coordinates(pose, 3, "pose")
        queue = []
        for cluster in self.summary():
            if cluster["pending"]:
                distance, angle = relative(pose, cluster["anchor_xy_m"])
                queue.append((-cluster["pending"], distance, abs(angle), cluster["id"]))
        if not queue:
            return None
        heapq.heapify(queue)
        pending, _, _, cid = heapq.heappop(queue)
        self._popped.add(cid)
        return CoverageCluster(cid, self._anchors[cid], self.cluster_radius_m,
                               -pending, self.pending_hints(cid))
