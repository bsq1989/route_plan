"""Replay verified directional motion on the native IR-SIM canvas."""

from bisect import bisect_right
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, Patch, Polygon, Wedge
from matplotlib.widgets import Button, Slider

from .app import configure_chinese_font, disable_environment_keyboard, make_environment_from_config
from .drawing import TargetCircles
from .scene import Rect


STAGES = {"initial": "原地发现", "turn": "原地转向", "approach": "定向渐近",
          "pickup": "执行局部计划", "pickup_stop": "停靠拾取"}
REASONS = {"all_collected": "全部清除", "no_precise_batch": "没有可拾取的精测批次",
           "no_observed_targets": "没有发现剩余目标", "no_safe_heading": "没有安全的候选朝向",
           "round_limit": "达到轮数上限", "Map boundary": "场地触边",
           "Motion circle boundary": "超出 auto_fence", "pickup_conflict": "拾取批次不符",
           "No complete pickup plan satisfies the motion deadband": "未找到完整拾取路线"}


class DirectionalViewer:
    def __init__(self, record, *, display=True):
        configure_chinese_font()
        self.record = record
        self.frames = record["trace"]
        self.targets_by_id = {str(key): point for key, point in record["initial"]["remaining"].items()}
        self.times = [frame["time_s"] for frame in self.frames]
        self.observation_indices = [frame["trace_index"] for frame in record["observations"]]
        self.plan_indices = [plan["trace_index"] for plan in record["plans"]]
        config = record["scene_config"]
        world, custom = config["world"], config["custom"]
        fence = custom.get("fence") if custom.get("fence_enabled", True) else None
        x, y = world.get("offset", (0., 0.))
        if fence:
            xs, ys = zip(*fence)
            bounds = min(xs), min(ys), max(xs), max(ys)
        else:
            bounds = x, y, x + world["width"], y + world["height"]
        display_config = {**config, "world": {**world, "plot": {**world.get("plot", {}), "show_title": False}}}
        self.env = make_environment_from_config(display_config, bounds, display)
        self.env.display = False  # The viewer's timer owns playback.
        self.fig, self.ax = self.env._env_plot.fig, self.env._env_plot.ax
        disable_environment_keyboard(self.env, self.fig.canvas)
        self.fig.canvas.manager.set_window_title("定向接近与分区拾取｜IR-SIM")
        self.fig.set_size_inches(10.4, 8.0, forward=True)
        self.fig.set_facecolor("#f1f4f9")
        self.fig.subplots_adjust(left=.12, right=.91, bottom=.17, top=.81)
        self.fig.suptitle("定向接近与分区拾取", x=.1, y=.965, ha="left", fontsize=21, color="#24334a")
        self.fig.text(.91, .95, "IR-SIM", ha="right", color="#52627b")
        self.status = self.fig.text(.1, .905, "", fontsize=11, color="#24334a")
        self.detail = self.fig.text(.1, .09, "", fontsize=10, color="#52627b")
        self.ax.set(xlim=bounds[::2], ylim=bounds[1::2], xlabel="map x / 米", ylabel="map y / 米")
        self.ax.set_axisbelow(True)
        self.ax.grid(color="#e2e8f0", linewidth=.6)
        if fence:
            self.ax.add_patch(Polygon(fence, fill=False, ec="#475569", lw=1.4, zorder=3))
        self.view = Wedge((0, 0), 1, 0, 90, fc="#72b8e2", alpha=.23, zorder=.6)
        self.sector = Wedge((0, 0), 1, 0, 60, width=.5, fc="#3bb693", alpha=.30, zorder=.8)
        self.circle = Circle((0, 0), 1, fill=False, ec="#8155af", ls="--", lw=1.4, zorder=2)
        self.safety = Polygon([[0, 0]], fill=False, ec="#b77720", ls="--", lw=1.25, zorder=4)
        self.collector = Polygon([[0, 0]], fc="#149973", ec="#08734e", lw=1.1, zorder=5)
        for artist in (self.view, self.sector, self.circle, self.safety, self.collector):
            self.ax.add_patch(artist)
        self.links = LineCollection([], linewidths=1.4, zorder=3)
        self.ax.add_collection(self.links)
        self.targets = TargetCircles(self.ax, facecolor="#f4ab45", edgecolor="#b56b14", linewidth=.7, zorder=5)
        self.picked = self.ax.scatter([], [], marker="x", color="#438a78", s=30, zorder=5)
        obstacles = TargetCircles(self.ax, facecolor="#526175", edgecolor="#344154", linewidth=.8, zorder=5)
        obstacles.set_paths([Circle((x, y), radius) for x, y, radius in record["initial"].get("obstacles", ())])
        self.path, = self.ax.plot([], [], color="#5678b7", linewidth=1.45, zorder=1.7)
        dims, tool = record["initial"]["dimensions"], custom["collector"]
        hx, hy = dims["body_length"] / 2, dims["body_width"] / 2
        m = dims["body_margin"]
        front, lateral = hx + tool["gap"], tool["lateral_offset"]
        self.safety_rect = Rect(-hx - m, -hy - m, hx + m, hy + m)
        self.collector_rect = Rect(front, lateral - tool["width"] / 2,
                                  front + tool["depth"], lateral + tool["width"] / 2)
        self.fig.legend(handles=[
            Patch(fc="#72b8e2", alpha=.35, label="发现扇区"), Patch(fc="#3bb693", alpha=.45, label="拾取圆环"),
            Patch(fill=False, ec="#8155af", ls="--", label="auto_fence"),
            Patch(fc="#f4ab45", label="目标圆"), Patch(fill=False, ec="#a96a28", ls="--", label="方位线索"),
            Patch(fc="#236db2", label="精测连线"), Patch(fc="#ce5149", label="当批目标"),
        ], loc="upper center", bbox_to_anchor=(.515, .872), ncol=7, fontsize=9, frameon=False)
        self.index = 0
        self.timer = self.fig.canvas.new_timer(interval=40)
        self.timer.add_callback(self.advance)
        self.playing = False
        self.controls = []
        if display:
            self.add_controls()
        self.draw(0)

    def add_controls(self):
        play = Button(self.fig.add_axes([.12, .032, .09, .037]), "播放")
        back = Button(self.fig.add_axes([.223, .032, .046, .037]), "←")
        step = Button(self.fig.add_axes([.28, .032, .046, .037]), "→")
        seek = Slider(self.fig.add_axes([.39, .036, .48, .026]), "进度", 0,
                      max(1, len(self.frames) - 1), valinit=0, valstep=1)
        play.on_clicked(lambda _: self.toggle())
        back.on_clicked(lambda _: self.seek(self.index - 1))
        step.on_clicked(lambda _: self.seek(self.index + 1))
        seek.on_changed(self.seek)
        self.controls = [play, back, step, seek]
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.fig.canvas.mpl_connect("close_event", lambda _: self.pause())

    def draw(self, index):
        self.index = index
        frame = self.frames[index]
        x, y, yaw = frame["pose"]
        self.env.robot.set_state(list(frame["pose"]))
        self.env.step()
        self.env.render()
        regions = self.record["regions"]
        heading = math.degrees(yaw)
        for patch, angle, radius in (
            (self.view, regions["view_angle_deg"], regions["view_range_m"]),
            (self.sector, regions["pickup_angle_deg"], regions["pickup_outer_radius_m"]),
        ):
            patch.set_center((x, y))
            patch.set_theta1(heading - angle / 2)
            patch.set_theta2(heading + angle / 2)
            patch.set_radius(radius)
        self.sector.set_width(regions["pickup_outer_radius_m"] - regions["pickup_inner_radius_m"])
        self.circle.set_visible(frame["auto_fence"] is not None)
        if frame["auto_fence"] is not None:
            cx, cy, radius = frame["auto_fence"]
            self.circle.set_center((cx, cy))
            self.circle.set_radius(radius)
        c, s = math.cos(yaw), math.sin(yaw)
        for patch, rect in ((self.safety, self.safety_rect), (self.collector, self.collector_rect)):
            patch.set_xy([(x + c*u - s*v, y + s*u + c*v) for u, v in rect.corners])

        initial = self.record["initial"]
        picked = set(map(str, frame["collected_ids"]))
        targets = self.targets_by_id
        picked_points = {tuple(targets[key]) for key in picked}
        self.targets.set_targets([point for key, point in targets.items() if key not in picked],
                                 initial["target_diameter"] / 2)
        self.picked.set_offsets([targets[key] for key in sorted(picked)] or [(math.nan, math.nan)])
        self.path.set_data([f["pose"][0] for f in self.frames[:index + 1]],
                           [f["pose"][1] for f in self.frames[:index + 1]])

        obs_index = bisect_right(self.observation_indices, index) - 1
        obs = self.record["observations"][obs_index] if obs_index >= 0 else {}
        plan_index = bisect_right(self.plan_indices, index) - 1
        batch = self.record["plans"][plan_index]["request"]["targets"] if frame["auto_fence"] and plan_index >= 0 else []
        batch_points = {(target["x"], target["y"]) for target in batch}
        picked_since_observation = obs and len(picked) != len(self.frames[obs["trace_index"]]["collected_ids"])
        rays, colors, styles = [], [], []
        for item in obs.get("known", ()):
            if item["quality"] == "bearing":
                if picked_since_observation:
                    continue
                ox, oy = item["origin_xy_m"]
                angle = item["bearing_map_rad"]
                span = regions["view_range_m"]
                rays.append([(ox, oy), (ox + span * math.cos(angle), oy + span * math.sin(angle))])
                colors.append("#a96a28")
                styles.append("--")
            else:
                point = tuple(item["xy_m"])
                if point in picked_points:
                    continue
                rays.append([(x, y), point])
                colors.append("#ce5149" if point in batch_points else
                              "#236db2" if item["quality"] == "precise" else "#a96a28")
                styles.append("-" if item["quality"] == "precise" else "--")
        self.links.set_segments(rays)
        self.links.set_colors(colors)
        self.links.set_linestyles(styles or ["-"])
        stage = STAGES[frame["stage"]]
        if index == len(self.frames) - 1:
            reason = self.record["stop_reason"]
            stage = REASONS.get(reason, reason)
        self.status.set_text(f"{self.record['case']} · {stage}    已拾取 {len(picked)} / {len(targets)}")
        self.detail.set_text(f"{frame['time_s']:.2f} s   ·   观测 {obs_index + 1} 帧   ·   朝向 {heading:.1f}°"
                             + (f"   ·   本批 {len(batch)} 件" if batch else ""))

    def on_key(self, event):
        if event.key == " ":
            self.toggle()
        elif event.key == "right":
            self.seek(self.index + 1)
        elif event.key == "left":
            self.seek(self.index - 1)
        elif event.key == "escape":
            self.pause()

    def pause(self):
        self.timer.stop()
        self.playing = False
        if self.controls:
            self.controls[0].label.set_text("播放")

    def toggle(self):
        if self.playing:
            self.pause()
        else:
            if self.index == len(self.frames) - 1:
                self.seek(0)
            self.playing = True
            self.controls[0].label.set_text("暂停")
            self.timer.start()
        self.fig.canvas.draw_idle()

    def seek(self, index, *, pause=True):
        if pause:
            self.pause()
        index = min(len(self.frames) - 1, max(0, int(index)))
        self.draw(index)
        slider = self.controls[-1]
        slider.eventson = False
        slider.set_val(index)
        slider.eventson = True
        self.fig.canvas.draw_idle()

    def advance(self):
        self.seek(self.index + 1, pause=False)
        if self.index == len(self.frames) - 1:
            self.pause()

    def save(self, gif=None, snapshot=None):
        if snapshot:
            path = Path(snapshot)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.draw(next((i for i, frame in enumerate(self.frames) if frame["stage"] == "pickup"), 0))
            self.fig.savefig(path, dpi=115)
        if gif:
            path = Path(gif)
            path.parent.mkdir(parents=True, exist_ok=True)
            fps = 10
            indices = {bisect_right(self.times, i / fps) - 1 for i in range(math.ceil(self.times[-1] * fps))}
            indices.update(i for i, frame in enumerate(self.frames) if frame["stage"] == "pickup_stop")
            with PillowWriter(fps=fps).saving(self.fig, str(path), dpi=75) as writer:
                for index in (0, *sorted(indices), len(self.frames) - 1):
                    self.draw(index)
                    hold = 5 if self.frames[index]["stage"] == "pickup_stop" else 1
                    for _ in range(hold):
                        writer.grab_frame()
                for _ in range(fps):
                    writer.grab_frame()

    def close(self):
        self.pause()
        self.env.end()


def show_replay(record, *, show=False, gif=None, snapshot=None):
    viewer = DirectionalViewer(record, display=show)
    try:
        if gif or snapshot:
            viewer.save(gif, snapshot)
        if show:
            viewer.seek(0)
            viewer.toggle()
            plt.show(block=True)
    finally:
        viewer.close()
