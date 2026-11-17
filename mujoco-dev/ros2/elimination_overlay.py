"""Triangulate D405 rings and draw them in the MuJoCo visualization scene."""

from math import isfinite

import mapbox_earcut
import mujoco
import numpy as np


FLOOR_OFFSET_M = 0.005
MAX_TRIANGLES = 8000
FILL = np.array([0.10, 0.90, 0.65, 0.45], dtype=np.float32)


def read_rings(document):
    if not isinstance(document, dict) or not isinstance(document.get("rings"), list) or not document["rings"]:
        raise ValueError("文件需要包含 rings[].points_m。")
    if document.get("frame_id", "base_link") != "base_link" or document.get("units", "m") != "m":
        raise ValueError("消除区需要使用 base_link 下的米制坐标。")

    polygons = {}
    holes = []
    for index, ring in enumerate(document["rings"]):
        points = ring.get("points_m") if isinstance(ring, dict) else None
        if not isinstance(points, list) or any(
            not isinstance(point, list) or len(point) < 2
            or not all(type(value) in (int, float) and isfinite(value) for value in point[:2])
            for point in points
        ):
            raise ValueError("points_m 的每个点需要包含有效的 XY。")
        xy = np.array([point[:2] for point in points], dtype=np.float64)
        if len(xy) and np.array_equal(xy[0], xy[-1]):
            xy = xy[:-1]
        if len(xy) < 3:
            raise ValueError("每条轮廓至少需要 3 个点。")

        role = ring.get("role", "outer")
        if role == "outer":
            polygons[index] = [xy]
        elif role == "hole":
            holes.append((ring.get("parent"), xy))
        else:
            raise ValueError("轮廓类型需要为 outer 或 hole。")

    for parent, xy in holes:
        if type(parent) is not int or parent not in polygons:
            raise ValueError("内孔需要指定对应的外轮廓。")
        polygons[parent].append(xy)
    return polygons.values()


class EliminationOverlay:
    def __init__(self, document):
        regions = []
        for rings in read_rings(document):
            points = np.concatenate(rings)
            ends = np.cumsum([len(ring) for ring in rings], dtype=np.uint32)
            indices = mapbox_earcut.triangulate_float64(points, ends).reshape(-1, 3)
            if not len(indices):
                raise ValueError("轮廓无法形成可填充的区域。")
            regions.append(points[indices])
        triangles = np.concatenate(regions)
        if len(triangles) > MAX_TRIANGLES:
            raise ValueError("消除区轮廓点过多。")

        edges = np.pad(triangles[:, 1:] - triangles[:, :1], ((0, 0), (0, 0), (0, 1)))
        normals = np.cross(edges[:, 0], edges[:, 1])
        reversed_winding = normals[:, 2] < 0
        edges[reversed_winding] = edges[reversed_winding, ::-1]
        normals[reversed_winding] *= -1
        axes = np.concatenate((edges, normals[:, None]), axis=1)
        lengths = np.linalg.norm(axes, axis=2)
        # Earcut can emit zero-area triangles along collinear runs.
        area = lengths[:, 2] > 0
        if not np.isfinite(lengths).all() or not area.any():
            raise ValueError("轮廓无法形成可填充的区域。")
        self.sizes = lengths[area]
        self.axes = (axes[area] / self.sizes[..., None]).transpose(0, 2, 1)
        self.origins = triangles[area, 0, :]

    def draw(self, scene, x, y, yaw):
        cosine, sine = np.cos(yaw), np.sin(yaw)
        rot = np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
        positions = np.column_stack((self.origins @ rot[:2, :2].T + (x, y),
                                     np.full(len(self.origins), FLOOR_OFFSET_M)))
        orientations = (rot @ self.axes).reshape(-1, 9)
        end = scene.ngeom + len(positions)
        for geom, size, position, orientation in zip(
            scene.geoms[scene.ngeom:end], self.sizes, positions, orientations, strict=True
        ):
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_TRIANGLE, size, position, orientation, FILL)
            geom.category = mujoco.mjtCatBit.mjCAT_DECOR
            geom.objid = -1
            geom.emission = 0.24
        scene.ngeom = end
