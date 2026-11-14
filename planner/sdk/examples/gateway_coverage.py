"""Run the SDK strategy with an asynchronous mock Gateway and no robot."""

import asyncio
import math

from route_plan_sdk import CoverageConfig, CoveragePolicy


class DemoGateway:
    def __init__(self):
        self.pose = (0., 0., 0.)
        self.point = (2.05, 0.)
        self.picked = False
        self.sequence = 0

    async def execute(self, action):
        await asyncio.sleep(0)
        self.sequence += 1
        result = {"status": "success", "time_s": float(self.sequence)}
        if action.kind == "observe":
            detections = []
            if not self.picked:
                x, y, _ = self.pose
                if math.dist((x, y), self.point) <= 1.4:
                    detections.append({"quality": "precise", "xy_m": self.point})
                else:
                    detections.append({
                        "quality": "bearing",
                        "origin_xy_m": (x, y),
                        "bearing_map_rad": math.atan2(self.point[1] - y, self.point[0] - x),
                    })
            result["observation"] = {"observation_id": self.sequence, "frame_id": "map",
                                     "detections": detections}
        else:
            confirmed = []
            stops = {stop.goal_index: stop.target_ids for stop in action.pickup_stops}
            for index, goal in enumerate(action.goals):
                self.pose = goal
                confirmed.extend(stops.get(index, ()))
            if action.kind == "pickup":
                self.picked = bool(confirmed)
                result["picked_ids"] = confirmed
        return {**result, "pose": self.pose}


async def main():
    gateway = DemoGateway()
    policy = CoveragePolicy(CoverageConfig(maximum_rounds=12), use_deferred_bearings=True)
    while not gateway.picked and (action := policy.next_action()) is not None:
        feedback = await gateway.execute(action)
        assert policy.submit_result(action.action_id, **feedback)
        print(action.kind, feedback["status"], feedback["pose"])
    assert gateway.picked, policy.stop_reason


if __name__ == "__main__":
    asyncio.run(main())
