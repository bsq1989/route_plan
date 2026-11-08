"""Physical-size target rendering shared by manual and planner viewers."""

from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle
import numpy as np


class TargetCircles(PatchCollection):
    """Circles in map metres; zoom and export preserve physical size."""

    def __init__(self, ax, **kwargs):
        super().__init__([], **kwargs)
        ax.add_collection(self)
        # Only archived point-model records use fixed-pixel markers.
        style = {key: value for key, value in kwargs.items() if key != "label"}
        self._legacy_points = ax.scatter([], [], s=30, label="_nolegend_", **style)
        self._legacy_points.set_visible(False)

    def set_targets(self, centres, radius):
        centres = list(centres)
        self._legacy_points.set_visible(radius == 0)
        if radius == 0:
            self.set_paths([])
            self._legacy_points.set_offsets(np.array(centres).reshape(-1, 2))
        else:
            self.set_paths([Circle((x, y), radius) for x, y in centres])
