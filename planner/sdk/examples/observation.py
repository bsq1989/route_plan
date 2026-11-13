"""Call once per stop using map-frame observations and the measured robot pose."""

import json

from route_plan_sdk import plan_observation_step


def main():
    request = {
        "request_id": "observation-demo",
        "pose": [0.0, 0.0, 0.0],
        "targets": [],
        "fence_enabled": False,
        "method": "stops",
        "use_ompl": False,
    }
    coarse = [{"id": "item-1", "x": 1.85, "y": 0.0, "error_radius": 0.15}]
    history = []
    remaining_hops = []
    observation_count = 0
    for _ in range(8):
        result = plan_observation_step(
            request, coarse, visited=history, pending_hops=remaining_hops,
            observations_since_pickup=observation_count,
        )
        print(json.dumps(result.to_dict(), ensure_ascii=False, allow_nan=False))
        if result.status == "pickup":
            assert result.pickup_stops[0].target_ids == ("item-1",)
            # In production execute goals through the stop index, trigger pickup
            # and wait for confirmation before removing the ID from observations.
            return
        assert result.status == "observe", result.reason
        # Replace the next two lines with ordered execution and measured odometry.
        history.append(request["pose"])
        request["pose"] = result.goals[-1]
        remaining_hops = result.pending_hops
        observation_count = result.observations_since_pickup
        # A stand-in for a fresh perception frame after stopping.
        if request["pose"][0] >= 0.50:
            request["targets"] = [{"id": "item-1", "x": 1.78, "y": 0.02}]
            remaining_hops = []  # Reliable coordinates changed. Replan from here.
    raise AssertionError("Example did not reach a pickup stop")


if __name__ == "__main__":
    main()
