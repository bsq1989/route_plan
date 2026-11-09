"""Map-frame pose commands, independent of the viewer and any task planner."""

from collections import deque
import math

from .scene import EPS, Scene, finite, normalize


class PoseController:
    """Turn toward the position, drive forward, then reach the final yaw.

    A new goal replaces the old goal from the current pose. Goal positions in
    the robot's current rear half-plane are rejected without changing the
    existing command. Public yaw values are in radians in the map frame.
    This controller does not plan detours around targets.
    """

    def __init__(self, scene: Scene, *, minimum_translation: float = 0., minimum_rotation: float = 0.):
        self.scene = scene
        self.minimum_translation = minimum_translation
        self.minimum_rotation = minimum_rotation
        self.goal: tuple[float, float, float] | None = None
        self.paused = False
        self._commands: deque[tuple[str, float]] = deque()

    @property
    def command(self) -> tuple[str, float] | None:
        return self._commands[0] if self._commands else None

    def set_goal(self, x: float, y: float, yaw: float) -> None:
        x, y, yaw = finite(x), finite(y), normalize(finite(yaw))
        if self.scene.failure:
            raise ValueError("Reset the scene after a collision")
        px, py, theta = self.scene.pose
        dx, dy = x - px, y - py
        if not (0 < x < self.scene.width and 0 < y < self.scene.height):
            raise ValueError("Goal must be inside the map")
        if dx * math.cos(theta) + dy * math.sin(theta) < -EPS:
            raise ValueError("Goal is behind the robot")
        distance = math.hypot(dx, dy)
        commands = []
        if distance > EPS:
            bearing = math.atan2(dy, dx)
            commands += [("turn", normalize(bearing - theta)), ("move", distance)]
            commands.append(("turn", normalize(yaw - bearing)))
        else:
            commands.append(("turn", normalize(yaw - theta)))
        commands = [(kind, amount) for kind, amount in commands if abs(amount) > EPS]
        for kind, amount in commands:
            minimum = self.minimum_translation if kind == "move" else self.minimum_rotation
            if minimum and abs(amount) <= minimum + EPS:
                raise ValueError(f"Goal requires a {kind} inside the motion deadband")
        self.goal = x, y, yaw
        self._commands = deque(commands)
        self.paused = False

    def stop(self) -> None:
        self._commands.clear()
        self.goal = None
        self.paused = False

    def step(self, dt: float | None = None) -> None:
        budget = finite(self.scene.config["world"]["step_time"] if dt is None else dt)
        if budget < 0:
            raise ValueError("Time step must be nonnegative")
        if self.scene.failure:
            self._commands.clear()
            return
        if self.paused:
            return
        while self._commands and budget > EPS:
            kind, remaining = self._commands[0]
            speed = float(self.scene.config["custom"]["forward_speed" if kind == "move" else "angular_speed"])
            amount = math.copysign(min(abs(remaining), speed * budget), remaining)
            self.scene.advance(**({"distance": amount} if kind == "move" else {"rotation": amount}))
            budget -= abs(amount) / speed
            remaining -= amount
            if self.scene.failure:
                self._commands.clear()
                return
            self._commands.popleft()
            if abs(remaining) > EPS:
                self._commands.appendleft((kind, remaining))
