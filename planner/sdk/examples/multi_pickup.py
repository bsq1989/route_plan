"""Three distinct, nonoverlapping disks serviced at one parking pose."""
from route_plan_sdk import PlanningRequest, Target, plan

result = plan(PlanningRequest(
    width=12, height=9, pose=(3, 4.5, 0), method="stops", timeout=3,
    targets=[Target("item-a", 5, 4.35), Target("item-b", 5.1, 4.5), Target("item-c", 5.2, 4.65)],
))
if not result.success:
    raise SystemExit(f"Planning failed: {result.status}: {result.reason}")
assert result.stop_count == 1
assert set(result.pickup_stops[0].target_ids) == {"item-a", "item-b", "item-c"}
print(f"{result.stop_count} pickup stop; {result.length:.3f} m")
for stop in result.pickup_stops:
    print(f"Reach goals[{stop.goal_index}]={stop.pose}, then pick {stop.target_ids}")
