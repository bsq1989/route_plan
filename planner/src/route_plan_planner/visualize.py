"""Render simulator replay; all geometry and collections come from Scene."""

from collections import deque
from pathlib import Path
import math

from route_plan_sim import PoseController, Scene


def _canvas(scene):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from route_plan_sim.app import configure_chinese_font

    configure_chinese_font()
    fig, ax = plt.subplots(figsize=(10, 7), dpi=95)
    fig.patch.set_facecolor("#f1f4f9")
    ax.set_facecolor("white")
    ax.set(xlim=(0, scene.width), ylim=(0, scene.height), aspect="equal",
           xlabel="横向位置 / 米", ylabel="纵向位置 / 米")
    ax.grid(alpha=.15)
    targets = ax.scatter([], [], color="#f5a623", edgecolors="#b77915", s=42, zorder=5, label="剩余目标")
    collected = ax.scatter([], [], color="#a3d8bd", marker="x", s=24, zorder=2, label="已消除")
    body = Polygon(scene.vertices(scene.body), color="#3568e8", alpha=.85, zorder=4)
    safety = Polygon(scene.vertices(scene.safety_body), fill=False, edgecolor="#c08a2f", linestyle="--", zorder=3)
    collector = Polygon(scene.vertices(scene.collector), color="#10b981", alpha=.4, zorder=4)
    for patch in (body, safety, collector):
        ax.add_patch(patch)
    path, = ax.plot([], [], color="#6f9de5", lw=1.8, zorder=2)
    heading, = ax.plot([], [], color="white", lw=2, zorder=6)
    ax.legend(loc="upper right", fontsize=9)
    title = ax.set_title("静态目标自动消除", pad=14, fontsize=15)
    fig.text(.5, .025, "蓝色：车身    橙色虚线：安全轮廓    绿色：前置消除区    回放采用仿真连续碰撞规则",
             ha="center", fontsize=9, color="#52627b")
    fig.tight_layout(rect=(0, .045, 1, 1))

    def update(current):
        import numpy as np
        targets.set_offsets(np.array(list(current.targets.values())).reshape(-1, 2))
        collected.set_offsets(np.array(list(current.collected.values())).reshape(-1, 2))
        body.set_xy(current.vertices(current.body))
        safety.set_xy(current.vertices(current.safety_body))
        collector.set_xy(current.vertices(current.collector))
        xs, ys = zip(*current.path)
        path.set_data(xs, ys)
        x, y, yaw = current.pose
        heading.set_data([x, x + current.body.xmax * math.cos(yaw)],
                         [y, y + current.body.xmax * math.sin(yaw)])
        state = "发生碰撞" if current.failure else "全部清除" if not current.targets else "自动执行中"
        prefix = f"随机种子 {current.seed}  ·  " if current.scenario == "random" else ""
        title.set_text(f"{prefix}{state}  ·  剩余 {len(current.targets)}  /  已消除 {len(current.collected)}")
        return current

    update(scene)
    return fig, update


def save_animation(scene: Scene, goals, output: Path, dt: float = .4, fps: int = 16):
    import matplotlib.pyplot as plt
    from matplotlib.animation import PillowWriter

    if not math.isfinite(dt) or dt <= 0 or fps < 1:
        raise ValueError("Animation dt and fps must be positive")
    trial = scene.clone()
    fig, update = _canvas(trial)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    controller = PoseController(trial)
    writer = PillowWriter(fps=fps)
    try:
        with writer.saving(fig, str(output), dpi=80):
            for _ in range(8):
                writer.grab_frame()
            for goal in goals:
                controller.set_goal(*goal)
                while controller.command is not None and not trial.failure:
                    controller.step(dt)
                    update(trial)
                    writer.grab_frame()
                if trial.failure:
                    break
            for _ in range(16):
                writer.grab_frame()
    finally:
        plt.close(fig)
    return trial


def save_snapshot(scene: Scene, output: Path):
    import matplotlib.pyplot as plt

    fig, _ = _canvas(scene)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        fig.savefig(output, dpi=130)
    finally:
        plt.close(fig)


def show(scene: Scene, goals, display=True):
    """Reuse IR-SIM's viewer; manual edits cancel the queued static plan."""
    from route_plan_sim.app import Viewer

    class PlanViewer(Viewer):
        def __init__(self):
            self.pending = deque(goals)
            super().__init__(scene.clone(), display=display)
            self.note("静态自动清除计划已加载。\n空格暂停，Esc 取消。")

        def set_goal(self, *args):
            self.pending.clear()
            super().set_goal(*args)

        def reset(self, scenario=None):
            self.pending.clear()
            super().reset(scenario)

        def apply_dimensions(self):
            self.pending.clear()
            super().apply_dimensions()

        def on_key(self, event):
            if event.key == "escape":
                self.pending.clear()
            super().on_key(event)

        def on_click(self, event):
            # Static replay has a fixed observation set; editing cancels it.
            if event.inaxes is self.ax:
                self.pending.clear()
                self.motion.stop()
            super().on_click(event)

        def tick(self):
            if self.pending and not self.command and not self.paused and not self.scene.failure:
                goal = self.pending.popleft()
                self.motion.set_goal(*goal)
                self._update_pose_fields(*goal)
                self.note(f"自动清除中，剩余 {len(self.scene.targets)} 个目标。\n空格暂停，Esc 取消。")
            super().tick()
            if not self.pending and not self.command and not self.scene.targets:
                self.note("全部目标已自动消除。")

    PlanViewer().run()
