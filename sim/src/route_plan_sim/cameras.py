"""Deterministic planar camera observations, separate from collision truth."""

from dataclasses import dataclass
import math
from random import Random

from .scene import EPS, Scene, normalize


def to_map(pose, point):
    x, y, yaw = pose
    forward, left = point
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * forward - s * left, y + s * forward + c * left


def to_local(pose, point):
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


@dataclass(frozen=True)
class Camera:
    frame_id: str
    mount: tuple[float, float, float]  # body x, y, yaw, in metres and radians
    horizontal_fov: float
    max_range: float
    wide: bool = False
    range_error_per_m: float = 0.
    merge_angle: float = 0.

    def __post_init__(self):
        if not all(map(math.isfinite, (*self.mount, self.horizontal_fov, self.max_range,
                                       self.range_error_per_m, self.merge_angle))):
            raise ValueError("Camera parameters must be finite")
        if not 0 < self.horizontal_fov <= math.pi or self.max_range <= 0:
            raise ValueError("Horizontal FOV must be in (0, 180] degrees and range must be positive")
        if self.range_error_per_m < 0 or not 0 <= self.merge_angle <= self.horizontal_fov:
            raise ValueError("Range error and merge angle must be nonnegative and merge angle must fit the FOV")

    def map_pose(self, robot_pose):
        return (*to_map(robot_pose, self.mount[:2]), normalize(robot_pose[2] + self.mount[2]))

    def capture(self, scene: Scene, timestamp_s: float, rng: Random | None = None):
        """Return an optical-frame observation and separate truth diagnostics."""
        pose = self.map_pose(scene.pose)
        visible, visibility = [], {}
        for key, point in scene.targets.items():
            local = to_local(pose, point)
            distance = math.hypot(*local)
            bearing = math.atan2(local[1], local[0])
            if distance > self.max_range + EPS:
                visibility[key] = "out_of_range"
            elif abs(bearing) > self.horizontal_fov / 2 + EPS:
                visibility[key] = "out_of_fov"
            else:
                blockers = (*scene.obstacles, *((*other, scene.target_radius)
                            for other_id, other in scene.targets.items() if other_id != key))
                visibility[key] = ("occluded" if self._blocked(pose, point, scene.target_radius, blockers)
                                   else "visible")
            if visibility[key] == "visible":
                visible.append((bearing, distance, local))

        if self.wide:
            detections = self._wide_detections(visible, rng)
        else:
            detections = [{"xy_m": list(local)} for _, _, local in visible]
        return {
            "timestamp_s": timestamp_s, "frame_id": self.frame_id,
            "camera_pose_map": list(pose), "detections": detections,
        }, visibility

    @staticmethod
    def _blocked(pose, point, target_radius, blockers):
        # Trace the centre ray up to the target's near surface.
        dx, dy = point[0] - pose[0], point[1] - pose[1]
        distance = math.hypot(dx, dy)
        if distance <= target_radius:
            return False
        ux, uy = dx / distance, dy / distance
        for x, y, radius in blockers:
            ox, oy = x - pose[0], y - pose[1]
            along = min(max(ox * ux + oy * uy, 0.), distance - target_radius)
            if math.hypot(ox - along * ux, oy - along * uy) <= radius + EPS:
                return True
        return False

    def _wide_detections(self, visible, rng):
        if self.range_error_per_m and rng is None:
            raise ValueError("A seeded random generator is required for noisy observations")
        groups = []
        for bearing, distance, _ in sorted(visible):
            if not groups or bearing - groups[-1][0][0] > self.merge_angle + EPS:
                groups.append([])
            groups[-1].append((bearing, distance))
        detections = []
        for group in groups:
            bearing = (group[0][0] + group[-1][0]) / 2
            distance = min(item[1] for item in group)
            if self.range_error_per_m:
                distance *= 1 + self.range_error_per_m * rng.uniform(.6, 1.)
            detections.append({"bearing_rad": bearing,
                               "xy_m": [distance * math.cos(bearing), distance * math.sin(bearing)]})
        return detections
