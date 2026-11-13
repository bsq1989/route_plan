"""Plan in map coordinates inside a polygon with a recessed corner."""

from route_plan_sdk import plan

result = plan({
    "frame_id": "map",
    "fence": [[-2, -1], [2, -1], [2, 1], [0, 1], [0, 2], [-2, 2]],
    "pose": [-1.2, 0, 0],
    "targets": [{"id": "cup", "x": 1.25, "y": 0}],
    "method": "stops",
})
if not result.success:
    raise SystemExit(f"{result.status}: {result.reason}")
print("Map pickup pose:", result.pickup_stops[0].pose)
