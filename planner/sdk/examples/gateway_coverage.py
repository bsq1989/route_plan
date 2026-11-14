"""Run the SDK strategy with an asynchronous mock Gateway and no robot."""

import argparse
import asyncio
import math

from route_plan_sdk import CoverageConfig, CoveragePolicy


class DemoGateway:
    def __init__(self, *, cluster):
        self.pose = (0., 0., 0.)
        self.point = (2.05, 0.)
        self.picked = False
        self.sequence = 0
        self.cluster = cluster

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
                elif self.cluster:
                    detections.append({"quality": "coarse", "xy_m": (2.1, 0.05)})
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


async def main(cluster=False):
    gateway = DemoGateway(cluster=cluster)
    config = CoverageConfig(maximum_rounds=12, cluster_radius_m=1.6 if cluster else None)
    policy = CoveragePolicy(config, use_deferred_bearings=not cluster)
    while (action := policy.next_action()) is not None:
        while policy.is_next_cluster():
            visit = policy.next_cluster()
            print("cluster", visit.cluster_id, visit.anchor_xy_m, visit.pending_count)
        feedback = await gateway.execute(action)
        if cluster and action.kind == "observe":
            supplemental = []
            if not gateway.picked and math.dist(gateway.pose[:2], gateway.point) > config.precision_range_m:
                supplemental.append({"quality": "coarse", "xy_m": (2.09, .03)})
            assert policy.submit_observation(
                action.action_id, pose=feedback["pose"], time_s=feedback["time_s"],
                observation=feedback["observation"], supplemental_detections=(supplemental,),
            )
        else:
            assert policy.submit_result(action.action_id, **feedback)
        print(action.kind, feedback["status"], feedback["pose"])
        if gateway.picked and not cluster:
            break
    assert gateway.picked, policy.stop_reason


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cluster", action="store_true")
    asyncio.run(main(parser.parse_args().cluster))
