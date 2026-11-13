"""Plan inside a circle centred on the current base_link position."""

import math

from route_plan_sdk import plan


def main():
    request = {
        "mode": "auto_fence",
        "method": "stops",
        "pose": [-2.0, 3.0, 0.0],
        "targets": [{"id": "cup", "x": -0.8, "y": 3.0}],
        "auto_fence_radius_m": 1.4,
        "use_ompl": False,
    }
    result = plan(request)
    assert result.success, result.reason
    assert result.expected_collected_ids == ["cup"]
    assert all(math.dist(request["pose"][:2], goal[:2]) <= request["auto_fence_radius_m"] + 1e-9
               for goal in result.goals)
    print(f"{result.stop_count} pickup stop; {result.length:.3f} m")

    limited = plan({**request, "auto_fence_radius_m": 0.30})
    assert not limited.success and not limited.goals
    print(f"0.30 m radius: {limited.status}")


if __name__ == "__main__":
    main()
