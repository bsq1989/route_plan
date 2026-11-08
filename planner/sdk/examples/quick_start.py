"""Copy this file into another project; only the installed SDK is imported."""

import json

from route_plan_sdk import PlanningRequest, Target, plan


def main():
    request = PlanningRequest(
        request_id="pickup-demo-001",
        frame_id="map",
        width=12.0,
        height=9.0,
        pose=(3.0, 4.5, 0.0),
        targets=[Target("object-101", 5.0, 4.5), Target("object-205", 6.5, 5.0)],
        target_diameter=0.10,
        method="stops",
        timeout=3.0,
    )
    response = plan(request)
    print(json.dumps(response.to_dict(), ensure_ascii=False, indent=2, allow_nan=False))
    if not response.success:
        raise SystemExit(f"Planning failed: {response.status}: {response.reason}")
    pickups = {stop.goal_index: stop for stop in response.pickup_stops}
    print(f"pickup stops={response.stop_count}; path={response.length:.3f} m")
    for index, step in enumerate(response.steps):
        print(f"goal[{index}]: map goal={step.goal}")
        if index in pickups:
            print(f"  after arrival, pick this batch: {pickups[index].target_ids}")
        # Integration point: send this goal to your controller and wait for its
        # actual completion before dispatching the next goal. This example does
        # not send hardware commands. At pickup checkpoints, confirm the batch
        # pickup before continuing. Expected pickups are model predictions.


if __name__ == "__main__":
    main()
