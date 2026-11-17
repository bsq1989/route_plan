"""Place numbered visual discs inside the arena without touching exclusion zones."""

from math import atan2, cos, sin

import mujoco
import numpy as np
import shapely
from shapely.geometry import GeometryCollection, LineString, Polygon


DISC_RADIUS = 0.025
DISC_HALF_HEIGHT = 0.001
MAX_DISCS = 100
DISC_SPACING = 0.15  # Leave room for the screen-space numbers.
DISC_COLOR = np.array([0.96, 0.55, 0.15, 1.0], dtype=np.float32)
UPRIGHT = np.eye(3, dtype=np.float64).reshape(9)


def inner_fence_bounds(model, data):
    bases = [
        geom for geom in range(model.ngeom)
        if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_CYLINDER
        and model.geom(geom).name.startswith("post_")
        and model.geom(geom).name.endswith("_left_base")
    ]
    if not bases:
        raise ValueError("场景缺少围栏圆底座。")
    radii = model.geom_size[bases, 0]
    centers = data.geom_xpos[bases, :2]
    lower, upper = [], []
    for coordinate in centers.T:
        near = np.isclose(coordinate, coordinate.min())
        far = np.isclose(coordinate, coordinate.max())
        lower.append(np.max(coordinate[near] + radii[near]))
        upper.append(np.min(coordinate[far] - radii[far]))
    return np.array(lower), np.array(upper)


class DiscScatter:
    def __init__(self, model, data, robot):
        self.lower, self.upper = inner_fence_bounds(model, data)
        self.floor_z = data.geom("floor").xpos[2]
        body = Polygon(robot["body"]["polygon"])
        self.safety_body = body.buffer(robot["safety"]["body_margin"], join_style="mitre")
        _, ymin, _, ymax = body.bounds
        front_x = robot["body"]["front_x"]
        self.nose = body.intersection(LineString([(front_x, ymin), (front_x, ymax)]))

    def exclusion(self, overlay):
        collectors = (Polygon(shell, holes) for shell, *holes in overlay.rings)
        # The envelope down to the front edge also covers the open gap behind each collector.
        return shapely.union_all([
            self.safety_body,
            *(GeometryCollection([collector, self.nose]).convex_hull for collector in collectors),
        ])

    def place(self, count, pose, overlay, rng):
        if type(count) is not int or not 1 <= count <= MAX_DISCS:
            raise ValueError(f"圆片数量应为 1 到 {MAX_DISCS} 的整数。")
        if pose is None:
            raise ValueError("正在等待小车位姿。")
        low, high = self.lower + DISC_RADIUS, self.upper - DISC_RADIUS
        if np.any(low >= high):
            raise ValueError("围栏内没有可撒布的区域。")

        x, y = pose["position"][:2]
        w, qx, qy, qz = pose["rotation"]
        yaw = atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        c, s = cos(yaw), sin(yaw)
        exclusion = self.exclusion(overlay)
        centers = []
        for _ in range(20):
            world = rng.uniform(low, high, (max(128, 4 * count), 2))
            local = (world - (x, y)) @ np.array([[c, -s], [s, c]])
            valid = ~shapely.dwithin(shapely.points(local), exclusion, DISC_RADIUS)
            for candidate in world[valid]:
                if centers and np.any(np.sum((centers - candidate) ** 2, axis=1) <= DISC_SPACING ** 2):
                    continue
                centers.append(candidate)
                if len(centers) == count:
                    return tuple(map(tuple, centers))
        raise ValueError("可用空间不足，无法撒布指定数量的圆片。")

    def draw(self, scene, centers):
        size = np.array([DISC_RADIUS, DISC_HALF_HEIGHT, 0.0])
        z = self.floor_z + 2 * DISC_HALF_HEIGHT
        end = scene.ngeom + len(centers)
        for number, (geom, (x, y)) in enumerate(zip(scene.geoms[scene.ngeom:end], centers, strict=True), 1):
            mujoco.mjv_initGeom(
                geom, mujoco.mjtGeom.mjGEOM_CYLINDER, size, np.array([x, y, z]), UPRIGHT, DISC_COLOR
            )
            geom.category = mujoco.mjtCatBit.mjCAT_DECOR
            geom.objid = -1
            geom.label = str(number)
        scene.ngeom = end
