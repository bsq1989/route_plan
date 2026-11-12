"""Simulated observations for a caller that switches between viewing and pickup."""

from math import atan2, cos, degrees, hypot, radians, remainder, sin, tau

from active_perception_sdk import ObservationCue, suggest_views

TARGETS = {"near": (0.8, 0.8), "behind_obstacle": (2.4, 0.0), "far": (4.0, -1.2)}
OBSTACLE_X, OBSTACLE_Y, OBSTACLE_RADIUS = 1.2, 0.0, 0.24
WIDE_FOV, LOCAL_FOV = radians(180), radians(30)
WIDE_RANGE, LOCAL_RANGE = 6.0, 1.5


def observe(origin, yaw, fov, max_range):
    ox, oy = OBSTACLE_X - origin[0], OBSTACLE_Y - origin[1]
    for target_id, (x, y) in TARGETS.items():
        dx, dy = x - origin[0], y - origin[1]
        distance = hypot(dx, dy)
        bearing = remainder(atan2(dy, dx) - yaw, tau)
        if distance == 0 or distance > max_range or abs(bearing) > fov / 2:
            continue
        fraction = (ox * dx + oy * dy) / (distance * distance)
        if 0 < fraction < 1 and hypot(ox - fraction * dx, oy - fraction * dy) <= OBSTACLE_RADIUS:
            continue
        yield target_id, bearing, distance


def main():
    origin = (0.0, 0.0)
    yaw = 0.0
    tried_headings = []
    print("Wide camera from start:", [item[0] for item in observe(origin, yaw, WIDE_FOV, WIDE_RANGE)])

    while True:
        visible = {target_id: TARGETS[target_id] for target_id, _, _ in observe(origin, yaw, LOCAL_FOV, LOCAL_RANGE)}
        if visible:
            print(f"Local camera confirmed {visible}. Hand off to pickup planning.")
            break
        tried_headings.append(yaw)

        cues = []
        for _, bearing, distance in observe(origin, yaw, WIDE_FOV, WIDE_RANGE):
            estimated_distance = distance * 1.25 if distance > LOCAL_RANGE else distance
            approximate_xy = (estimated_distance * cos(bearing), estimated_distance * sin(bearing))
            cues.append(ObservationCue(bearing, approximate_xy))
        candidates = suggest_views(
            cues,
            horizontal_fov=LOCAL_FOV,
            tried_yaw_offsets=(heading - yaw for heading in tried_headings),
        )
        if not candidates:
            print("No remaining observation direction in this scan.")
            break
        # For this observation demo, advance the simulated yaw directly.
        # A vehicle integration checks the sweep before commanding a turn.
        view = candidates[0]
        yaw += view.yaw_offset
        print(f"Observe at {degrees(yaw):.1f} degrees, estimated distance {view.estimated_nearest_distance_m:.2f} m")

    # An independent observation at a second pose demonstrates the occlusion.
    other_view = observe((2.0, 0.8), radians(-60), LOCAL_FOV, LOCAL_RANGE)
    print("Local camera from second pose:", {item[0]: TARGETS[item[0]] for item in other_view})


if __name__ == "__main__":
    main()
