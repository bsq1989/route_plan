"""Select observation headings from horizontal camera bearings."""

from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass
import math
from numbers import Real

__all__ = ["ObservationCue", "ViewSuggestion", "suggest_views"]

_ANGLE_TOLERANCE = 1e-12


def _finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        angle = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(angle):
        raise ValueError(f"{name} must be a finite number")
    return angle


def _signed_angle(angle: float) -> float:
    angle = math.remainder(angle, math.tau)
    return -math.pi if angle == math.pi else angle


@dataclass(frozen=True)
class ObservationCue:
    """A horizontal bearing in radians and an optional approximate position in meters."""

    bearing: float
    approximate_xy: tuple[float, float] | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "bearing", _signed_angle(_finite(self.bearing, "bearing")))
        if self.approximate_xy is not None:
            try:
                x, y = self.approximate_xy
            except (TypeError, ValueError) as error:
                raise ValueError("approximate_xy must contain x and y in meters") from error
            xy = (_finite(x, "approximate_xy.x"), _finite(y, "approximate_xy.y"))
            if not math.isfinite(math.hypot(*xy)):
                raise ValueError("approximate_xy must have a finite distance")
            object.__setattr__(self, "approximate_xy", xy)


@dataclass(frozen=True)
class ViewSuggestion:
    """A camera yaw change, its input cue indices, and an optional distance estimate."""

    yaw_offset: float
    cue_indices: tuple[int, ...]
    estimated_nearest_distance_m: float | None


def suggest_views(
    cues: Iterable[ObservationCue],
    *,
    horizontal_fov: float,
    tried_yaw_offsets: Iterable[float] = (),
) -> tuple[ViewSuggestion, ...]:
    """Return observation views ordered by absolute yaw change.

    Bearings and previous view headings are relative to the local camera's
    current viewing direction. Only reuse tried headings at the same observation
    position. A yaw change of 0 means to observe at the current heading.
    Bearings select the views. Approximate positions supply distance estimates.
    """
    fov = _finite(horizontal_fov, "horizontal_fov")
    if not 0 < fov <= math.tau:
        raise ValueError("horizontal_fov must be in (0, 2*pi]")

    tried = tuple(_signed_angle(_finite(yaw, "tried_yaw_offsets")) for yaw in tried_yaw_offsets)
    observations = tuple(cues)
    half_fov = fov / 2 + _ANGLE_TOLERANCE
    remaining = [
        (cue.bearing, index) for index, cue in enumerate(observations)
        if all(abs(_signed_angle(cue.bearing - yaw)) > half_fov for yaw in tried)
    ]

    def make_view(yaw: float, indices: list[int]) -> ViewSuggestion:
        positions = (observations[index].approximate_xy for index in indices)
        distance = min((math.hypot(*xy) for xy in positions if xy is not None), default=None)
        return ViewSuggestion(yaw, tuple(sorted(indices)), distance)

    in_view = [index for bearing, index in remaining if abs(bearing) <= half_fov]
    suggestions = [make_view(0.0, in_view)] if in_view else []
    # Cut the circle at the current heading so that rear bearings stay adjacent.
    angles = deque(sorted((bearing % math.tau, index) for bearing, index in remaining if abs(bearing) > half_fov))
    while angles:
        if angles[0][0] <= math.tau - angles[-1][0]:
            start, index = angles.popleft()
            end = start
            indices = [index]
            while angles and angles[0][0] - start <= fov + _ANGLE_TOLERANCE:
                end, index = angles.popleft()
                indices.append(index)
        else:
            end, index = angles.pop()
            start = end
            indices = [index]
            while angles and end - angles[-1][0] <= fov + _ANGLE_TOLERANCE:
                start, index = angles.pop()
                indices.append(index)
        suggestions.append(make_view(_signed_angle((start + end) / 2), indices))

    return tuple(sorted(suggestions, key=lambda view: (abs(view.yaw_offset), view.yaw_offset)))
