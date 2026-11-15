"""A simple polygon boundary and continuous contact with a moving rectangle."""

from dataclasses import dataclass, field
from functools import lru_cache
import math

EPS = 1e-9
Point = tuple[float, float]


def trig_times(a: float, b: float, value: float, delta: float) -> list[float]:
    """Solve a*cos(delta*t) + b*sin(delta*t) = value, 0 <= t <= 1."""
    radius = math.hypot(a, b)
    if radius < EPS or abs(delta) < EPS or abs(value) > radius + EPS:
        return []
    phase = math.atan2(b, a)
    alpha = math.acos(max(-1.0, min(1.0, value / radius)))
    low, high = sorted((0.0, delta))
    result = []
    for root in (phase - alpha, phase + alpha):
        for k in range(math.ceil((low - root - EPS) / math.tau),
                       math.floor((high - root + EPS) / math.tau) + 1):
            t = (root + k * math.tau) / delta
            if -EPS <= t <= 1 + EPS:
                result.append(max(0.0, min(1.0, t)))
    return result


@dataclass(frozen=True)
class Edge:
    start: Point
    end: Point
    length: float
    direction: Point

    @classmethod
    def between(cls, start, end):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if not math.isfinite(length) or length <= EPS:
            raise ValueError("fence edges must have finite, distinct endpoints")
        return cls(start, end, length, (dx / length, dy / length))

    def project(self, x, y):
        dx, dy = x - self.start[0], y - self.start[1]
        ux, uy = self.direction
        return ux * dx + uy * dy, ux * dy - uy * dx

    def distance(self, point):
        along, across = self.project(*point)
        return math.hypot(max(0., -along, along - self.length), across)

    def intersects(self, other):
        if any(self.distance(p) <= EPS for p in (other.start, other.end)):
            return True
        if any(other.distance(p) <= EPS for p in (self.start, self.end)):
            return True
        a, b = (self.project(*p)[1] for p in (other.start, other.end))
        c, d = (other.project(*p)[1] for p in (self.start, self.end))
        return a * b < 0 and c * d < 0


def _hits_rect(a, b, rect):
    """Clip a closed segment to the rectangle in its own coordinate frame."""
    first, last = 0., 1.
    for start, end, low, high in zip(a, b, (rect.xmin, rect.ymin), (rect.xmax, rect.ymax)):
        low, high = low - EPS, high + EPS
        change = end - start
        if change == 0:
            if not low <= start <= high:
                return False
            continue
        enter, leave = sorted(((low - start) / change, (high - start) / change))
        first, last = max(first, enter), min(last, leave)
        if first > last:
            return False
    return True


@dataclass(frozen=True)
class Fence:
    vertices: tuple[Point, ...]
    edges: tuple[Edge, ...] = field(init=False)
    bounds: tuple[float, float, float, float] = field(init=False)

    @classmethod
    def from_points(cls, points):
        if not isinstance(points, (list, tuple)):
            raise ValueError("fence must be a sequence of finite [x, y] pairs")
        try:
            vertices = []
            for point in points:
                if not isinstance(point, (list, tuple)) or len(point) != 2:
                    raise ValueError
                x, y = point
                if isinstance(x, bool) or isinstance(y, bool):
                    raise ValueError
                vertex = float(x), float(y)
                if not all(map(math.isfinite, vertex)):
                    raise ValueError
                vertices.append(vertex)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("fence must be a sequence of finite [x, y] pairs") from error
        return cls(tuple(vertices))

    def __post_init__(self):
        points = self.vertices
        if len(points) > 1 and points[0] == points[-1]:
            points = points[:-1]
        if len(points) < 3 or len(set(points)) != len(points):
            raise ValueError("fence must contain at least three distinct vertices")
        edges = tuple(Edge.between(a, b) for a, b in zip(points, (*points[1:], points[0])))
        for i, edge in enumerate(edges):
            previous = edges[i - 1]
            if edge.distance(previous.start) <= EPS or previous.distance(edge.end) <= EPS:
                raise ValueError("fence edges must not overlap")
            first = 1 if i == len(edges) - 1 else 0
            for other in edges[first:i - 1] if i > 1 else ():
                if edge.intersects(other):
                    raise ValueError("fence must be a simple polygon without self-intersections")
        xs, ys = zip(*points)
        bounds = min(xs), min(ys), max(xs), max(ys)
        # Use a local origin so a large map translation does not corrupt area.
        ox, oy = points[0]
        area = math.fsum((a[0] - ox) * (b[1] - oy) - (a[1] - oy) * (b[0] - ox)
                         for a, b in zip(points, (*points[1:], points[0])))
        if not math.isfinite(area) or abs(area) <= EPS**2:
            raise ValueError("fence must enclose a finite nonzero area")
        object.__setattr__(self, "vertices", points)
        object.__setattr__(self, "edges", edges)
        object.__setattr__(self, "bounds", bounds)

    def contains_disk(self, x, y, radius=0.):
        inside = False
        for edge in self.edges:
            if edge.distance((x, y)) <= radius + EPS:
                return False
            ax, ay = edge.start
            bx, by = edge.end
            if (ay > y) != (by > y) and x < ax + (bx - ax) * ((y - ay) / (by - ay)):
                inside = not inside
        return inside

    def first_contact(self, rect, pose, distance=0., rotation=0.):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        if isinstance(rect, PolygonFootprint):
            half = distance / 2
            # This disk encloses the whole sweep for a turn or straight move.
            if self.contains_disk(x + half * c, y + half * s, rect.reach + half):
                return math.inf
        corners = [(c * u - s * v, s * u + c * v) for u, v in rect.corners]
        u, v = corners[0]
        if not self.contains_disk(x + u, y + v):
            return 0.

        # A concave edge can cut the footprint even with all four corners inside.
        local = []
        for vx, vy in self.vertices:
            dx, dy = vx - x, vy - y
            local.append((c * dx + s * dy, -s * dx + c * dy))
        segments = zip(local, (*local[1:], local[0]))
        if any(rect.intersects_segment(a, b) if isinstance(rect, PolygonFootprint) else _hits_rect(a, b, rect)
               for a, b in segments):
            return 0.
        if not distance and not rotation:
            return math.inf

        # A first contact has a vertex on an edge (including collinear overlap).
        # Test stationary fence vertices in the rectangle's moving frame too.
        first = min(rect.first_contact(u, v, distance, rotation) for u, v in local)
        for edge in self.edges:
            ux, uy = edge.direction
            base_along, base_across = edge.project(x, y)
            rate = distance * (ux * s - uy * c)
            if distance and not rate:
                continue
            for u, v in corners:
                across = ux * v - uy * u
                along = ux * u + uy * v
                if distance:
                    times = [-(base_across + across) / rate]
                else:
                    times = trig_times(across, along, -base_across, rotation)
                for t in times:
                    if not -EPS <= t <= min(1 + EPS, first):
                        continue
                    t = max(0., min(1., t))
                    if distance:
                        position = base_along + along + distance * t * (ux * c + uy * s)
                    else:
                        angle = rotation * t
                        position = base_along + along * math.cos(angle) - across * math.sin(angle)
                    if -EPS <= position <= edge.length + EPS:
                        first = min(first, t)
        return first


@dataclass(frozen=True)
class PolygonFootprint:
    """Closed simple polygon with analytic disk contact under straight/turn motion."""

    polygon: Fence
    reach: float = field(init=False)

    def __post_init__(self):
        object.__setattr__(self, "reach", max(math.hypot(*point) for point in self.corners))

    @classmethod
    def from_points(cls, points):
        try:
            return _polygon_footprint(tuple(tuple(point) for point in points))
        except (TypeError, ValueError) as error:
            raise ValueError(f"Invalid collector polygon: {error}") from error

    @property
    def corners(self):
        return self.polygon.vertices

    @property
    def xmin(self):
        return self.polygon.bounds[0]

    @property
    def ymin(self):
        return self.polygon.bounds[1]

    @property
    def xmax(self):
        return self.polygon.bounds[2]

    @property
    def ymax(self):
        return self.polygon.bounds[3]

    def intersects_segment(self, a, b):
        if not _hits_rect(a, b, self):
            return False
        edge = Edge.between(a, b)
        return self.contains(*a) or any(edge.intersects(other) for other in self.polygon.edges)

    def contains(self, x, y):
        return self.intersects_circle(x, y, 0.)

    @lru_cache(maxsize=16)
    def outward_offset(self, margin):
        """Offset every face of a convex footprint by a perpendicular margin."""
        edges = self.polygon.edges
        turns = [a.direction[0] * b.direction[1] - a.direction[1] * b.direction[0]
                 for a, b in zip(edges, (*edges[1:], edges[0]))]
        winding = 1 if turns[0] > 0 else -1
        if any(turn * winding <= EPS for turn in turns):
            raise ValueError("Body polygon must be strictly convex")
        if not margin:
            return self
        normals = [(winding * uy, -winding * ux) for ux, uy in (edge.direction for edge in edges)]
        points = []
        for index, (x, y) in enumerate(self.corners):
            ax, ay = normals[index - 1]
            bx, by = normals[index]
            scale = margin / (1 + ax * bx + ay * by)
            points.append((x + scale * (ax + bx), y + scale * (ay + by)))
        return self.from_points(points)

    def turn_preserves_clearance(self, px, py, angle, contact_distance=None):
        """Check distances at every closest-feature transition and extremum."""
        times = {0., 1.}
        for edge in self.polygon.edges:
            ux, uy = edge.direction
            ax, ay = edge.start
            along, across = ux * px + uy * py, uy * px - ux * py
            start_along = ux * ax + uy * ay
            for projection in (start_along, start_along + edge.length):
                times.update(trig_times(along, -across, projection, angle))
            times.update(trig_times(across, along, uy * ax - ux * ay, angle))
            times.update(trig_times(along, -across, 0., angle))
            times.update(trig_times(ax * py - ay * px, -ax * px - ay * py, 0., angle))
        largest = 0.
        for t in sorted(times):
            c, s = math.cos(angle * t), math.sin(angle * t)
            point = c * px + s * py, -s * px + c * py
            distance = (0. if self.contains(*point) else
                        min(edge.distance(point) for edge in self.polygon.edges))
            if distance + EPS < largest:
                return False
            if contact_distance is not None and distance <= contact_distance < largest:
                return False
            largest = max(distance, largest)
        return True

    def forward_preserves_clearance(self, x, y):
        """Moving the vehicle forward moves a local point along negative x."""
        if self.contains(x, y):
            return True
        distance, closest_x = math.inf, 0.
        for edge in self.polygon.edges:
            along, across = edge.project(x, y)
            clipped = min(edge.length, max(0., along))
            gap = math.hypot(along - clipped, across)
            if gap < distance:
                distance = gap
                closest_x = edge.start[0] + clipped * edge.direction[0]
        return x <= closest_x + EPS

    def longitudinal_spans(self, y):
        """Closed forward-axis intervals inside the polygon at lateral offset y."""
        crossings, spans = [], []
        for edge in self.polygon.edges:
            ax, ay = edge.start
            bx, by = edge.end
            if (ay > y) != (by > y):
                crossings.append(ax + (bx - ax) * ((y - ay) / (by - ay)))
            elif abs(ay - y) <= EPS and abs(by - y) <= EPS:
                spans.append((min(ax, bx), max(ax, bx)))
        crossings.sort()
        spans.extend(zip(crossings[::2], crossings[1::2]))
        merged = []
        for left, right in sorted(spans):
            if merged and left <= merged[-1][1] + EPS:
                merged[-1] = (merged[-1][0], max(merged[-1][1], right))
            else:
                merged.append((left, right))
        return merged

    @property
    @lru_cache(maxsize=16)
    def pickup_samples(self):
        """Interior points at several depths and lateral offsets for pose search."""
        result = []
        for fraction in (.8, .6, .4, .2):
            y = self.ymin + fraction * (self.ymax - self.ymin)
            for left, right in self.longitudinal_spans(y):
                if right - left > EPS:
                    result.extend((left + part * (right - left), y) for part in (.85, .5))
        return tuple(result)

    def intersects_circle(self, x, y, radius):
        if (x < self.xmin - radius - EPS or x > self.xmax + radius + EPS or
                y < self.ymin - radius - EPS or y > self.ymax + radius + EPS):
            return False
        inside = False
        for edge in self.polygon.edges:
            ax, ay = edge.start
            bx, by = edge.end
            if edge.distance((x, y)) <= radius + EPS:
                return True
            if (ay > y) != (by > y) and x < ax + (bx - ax) * ((y - ay) / (by - ay)):
                inside = not inside
        return inside

    def first_contact(self, px, py, distance, rotation, radius=0.):
        if distance:
            if (px < self.xmin - radius - EPS or px - distance > self.xmax + radius + EPS
                    or py < self.ymin - radius - EPS or py > self.ymax + radius + EPS):
                return math.inf
        elif math.hypot(px, py) > self.reach + radius + EPS:
            return math.inf
        if self.intersects_circle(px, py, radius):
            return 0.
        if not distance and not rotation:
            return math.inf
        first = math.inf
        for edge in self.polygon.edges:
            ux, uy = edge.direction
            ax, ay = edge.start
            across = ux * py - uy * px
            along = ux * px + uy * py
            edge_origin = ux * ay - uy * ax
            for offset in (-radius, radius) if radius else (0.,):
                value = edge_origin + offset
                if distance:
                    rate = uy * distance
                    times = [(value - across) / rate] if abs(rate) > EPS else ()
                else:
                    times = trig_times(across, -along, value, rotation)
                for t in times:
                    if not -EPS <= t <= min(first, 1. + EPS):
                        continue
                    theta = rotation * t
                    x = math.cos(theta) * px + math.sin(theta) * py - distance * t
                    y = -math.sin(theta) * px + math.cos(theta) * py
                    projection, _ = edge.project(x, y)
                    if -EPS <= projection <= edge.length + EPS:
                        first = max(0., min(1., t))
            # End caps of the disk-offset segment. These also cover collinear
            # zero-radius entries into an edge during a straight move.
            if distance:
                dy = abs(py - ay)
                if dy <= radius + EPS:
                    root = math.sqrt(max(0., radius**2 - dy**2))
                    times = ((px - ax - root) / distance, (px - ax + root) / distance)
                else:
                    times = ()
            elif radius:
                value = (px**2 + py**2 + ax**2 + ay**2 - radius**2) / 2
                times = trig_times(px * ax + py * ay, py * ax - px * ay, value, rotation)
            else:
                times = ()
            for t in times:
                if -EPS <= t <= min(first, 1. + EPS):
                    first = max(0., min(1., t))
        return first


@lru_cache(maxsize=16)
def _polygon_footprint(points):
    return PolygonFootprint(Fence.from_points(points))
