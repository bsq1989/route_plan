"""Independent recovery for targets already inside the vehicle safety margin."""

from dataclasses import dataclass
import math
from typing import Mapping

from route_plan_sim.scene import EPS, normalize, trig_times

from .api import PlanningRequest, Pose, TargetId, _empty_scene


@dataclass(frozen=True)
class EscapeResponse:
    status: str
    frame_id: str
    request_id: str | None
    initial_intrusion_ids: tuple[TargetId, ...]
    goals: tuple[Pose, ...] = ()
    final_pose: Pose | None = None
    reason: str | None = None

    @property
    def success(self) -> bool:
        return self.status in ("success", "already_clear")

    def to_dict(self):
        return {
            "status": self.status,
            "success": self.success,
            "frame_id": self.frame_id,
            "request_id": self.request_id,
            "initial_intrusion_ids": list(self.initial_intrusion_ids),
            "goals": [list(goal) for goal in self.goals],
            "final_pose": list(self.final_pose) if self.final_pose is not None else None,
            "reason": self.reason,
        }


def _turn_moves_away(rect, px, py, angle, contact_distance=None):
    """Check extrema of the continuous point-to-rectangle distance.

    Split at face transitions and at stationary points for the side and corner
    regions. Distance is monotone between these times.
    """
    times = {0., 1.}
    for edge in (rect.xmin, rect.xmax, 0.):
        times.update(trig_times(px, py, edge, angle))
    for edge in (rect.ymin, rect.ymax, 0.):
        times.update(trig_times(py, -px, edge, angle))
    for cx, cy in rect.corners:
        times.update(trig_times(cx * py - cy * px, -cx * px - cy * py, 0., angle))
    largest_distance = 0.
    for time in sorted(times):
        c, s = math.cos(angle * time), math.sin(angle * time)
        x, y = c * px + s * py, -s * px + c * py
        distance = math.hypot(max(rect.xmin - x, 0., x - rect.xmax),
                              max(rect.ymin - y, 0., y - rect.ymax))
        if distance + EPS < largest_distance:
            return False
        if contact_distance is not None and distance <= contact_distance < largest_distance:
            return False
        largest_distance = max(distance, largest_distance)
    return True


def _forward_exit(scene, points, initial_intrusions):
    """Find a forward endpoint past the last intrusion and before new contact."""
    body, safety, radius = scene.body, scene.safety_body, scene.target_radius
    local = [scene.local_point(*point) for point in points]
    # Forward motion decreases local x. Even an intrusion cleared by the
    # preceding turn must keep its body clearance for the whole escape.
    if any(local[i][0] > body.xmax for i in initial_intrusions):
        return None
    intrusions = {i for i in initial_intrusions if safety.intersects_circle(*local[i], radius)}
    exit_distance = 0.
    for i in intrusions:
        x, y = local[i]
        dy = max(safety.ymin - y, 0., y - safety.ymax)
        reach = math.sqrt(max(0., (radius + EPS)**2 - dy**2))
        exit_distance = max(exit_distance, x - safety.xmin + reach)

    # Prefer a centimetre past clearance. Use a shorter move if another
    # obstacle or the map boundary limits the available space.
    distance = exit_distance + .01
    first_contact = scene._wall_contact(distance, 0.)
    for i, (x, y) in enumerate(local):
        rect = body if i in intrusions else safety
        first_contact = min(first_contact, rect.first_contact(x, y, distance, 0., radius))
    if math.isfinite(first_contact):
        distance = (exit_distance + first_contact * distance) / 2
    if distance <= exit_distance + EPS:
        return None
    if any(safety.intersects_circle(x - distance, y, radius) for x, y in local):
        return None
    x, y, yaw = scene.pose
    return x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw


def plan_escape(request: PlanningRequest | Mapping) -> EscapeResponse:
    """Find a local exit from a pre-existing safety margin intrusion.

    Searches a forward move or an in-place turn followed by optional forward
    motion. The affected disks must start clear of the physical body. Every
    move preserves their clearance, with the usual safety margin for other
    disks and the usual map checks. All supplied disks remain obstacles.
    """
    request = request if isinstance(request, PlanningRequest) else PlanningRequest.from_dict(request)
    scene = _empty_scene(request)
    body, safety, radius = scene.body, scene.safety_body, scene.target_radius
    points = [(target.x, target.y) for target in request.targets]
    for x, y in points:
        if not (radius + EPS < x < scene.width - radius - EPS
                and radius + EPS < y < scene.height - radius - EPS):
            raise ValueError("Target must be inside the map")
    local = [scene.local_point(*point) for point in points]
    intrusions = {i for i, point in enumerate(local) if safety.intersects_circle(*point, radius)}
    ids = tuple(target.id for i, target in enumerate(request.targets) if i in intrusions)

    def response(status, goals=(), final_pose=None, reason=None):
        return EscapeResponse(status, request.frame_id, request.request_id, ids, goals, final_pose, reason)

    if not intrusions:
        return response("already_clear", final_pose=scene.pose)
    for i in sorted(intrusions):
        if body.intersects_circle(*local[i], radius):
            return response("body_contact", reason=f"Target {request.targets[i].id!r} touches the physical body")

    goal = _forward_exit(scene, points, intrusions)
    if goal is not None:
        return response("success", (goal,), goal)

    x, y, yaw = scene.pose
    turned = scene.clone()
    for degrees in range(5, 180, 5):
        for direction in (1, -1):
            heading = normalize(yaw + math.radians(degrees * direction))
            angle = normalize(heading - yaw)
            if math.isfinite(scene._wall_contact(0., angle)):
                continue
            if any(math.isfinite((body if i in intrusions else safety).first_contact(px, py, 0., angle, radius))
                   for i, (px, py) in enumerate(local)):
                continue
            if any(not _turn_moves_away(body, *local[i], angle)
                   or not _turn_moves_away(safety, *local[i], angle, radius + EPS)
                   for i in intrusions):
                continue
            turned.pose = (x, y, heading)
            remaining = {i for i in intrusions
                         if safety.intersects_circle(*turned.local_point(*points[i]), radius)}
            if not remaining:
                return response("success", (turned.pose,), turned.pose)
            goal = _forward_exit(turned, points, intrusions)
            if goal is not None:
                return response("success", (turned.pose, goal), goal)
    return response("blocked", reason="No separating turn or forward move found")
