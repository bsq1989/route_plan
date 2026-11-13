"""IR-SIM viewer and manual controls; no task planning policy lives here."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import textwrap

import irsim
from irsim.lib import GeometryFactory
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.backend_bases import MouseButton
from matplotlib.patches import FancyBboxPatch, Patch, Polygon
from matplotlib.widgets import Button, RadioButtons, TextBox
import numpy as np
import yaml

from .scene import Scene, finite
from .control import PoseController
from .drawing import TargetCircles

OUTPUT = Path(__file__).resolve().parents[2] / "output"
INK = "#24334a"
MUTED = "#748399"
BLUE = "#3568e8"


def configure_chinese_font():
    """Prefer an installed CJK font on macOS, Windows and Linux."""
    preferred = ("PingFang SC", "Microsoft YaHei", "Noto Sans CJK SC",
                 "Source Han Sans SC", "WenQuanYi Micro Hei", "Heiti SC", "SimHei",
                 "Arial Unicode MS")
    installed = {font.name for font in font_manager.fontManager.ttflist}
    available = [name for name in preferred if name in installed]
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = available + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def make_environment(scene: Scene, display: bool):
    """Give IR-SIM the resolved map and the same vehicle geometry as Scene."""
    with TemporaryDirectory(prefix="route-plan-scene-") as directory:
        resolved = Path(directory) / "world.yaml"
        resolved.write_text(yaml.safe_dump(scene.config, allow_unicode=True), encoding="utf-8")
        return irsim.make(str(resolved), display=display, step_mode="external", log_level="ERROR")


def error_text(error: ValueError) -> str:
    # Keep episode API errors stable while showing Chinese in the viewer.
    return {
        "Values must be finite numbers": "请输入有效的有限数值。",
        "Reset the scene after a collision": "发生碰撞，请先重置场景。",
        "Target must be inside the map": "整个目标圆必须位于场地内部。",
        "Target circles must not overlap": "目标圆不能与已有目标重叠。",
        "Cannot place a target inside the robot body": "目标圆不能触碰车身安全轮廓。",
        "Dimensions must be positive": "车身和消除区的尺寸必须大于 0。",
        "Body margin must be nonnegative": "车身外扩距离必须大于或等于 0。",
        "Resized footprint must fit inside the map": "新尺寸在当前位置或初始位置越界，请缩小尺寸。",
        "Resized body overlaps an uncollected target": "新安全轮廓覆盖了未消除点，请缩小尺寸或外扩值。",
        "Goal must be inside the map": "目标位置必须位于场地内部。",
        "Goal is behind the robot": "目标位置在车后方，无法接受此指令。",
        "Reverse motion is disabled": "小车不能倒车，请设置前方目标。",
    }.get(str(error), f"操作失败：{error}")


def failure_text(failure: str | None) -> str | None:
    if failure == "Map boundary":
        return "安全轮廓或消除器触碰场地边界，已停止"
    if failure and failure.startswith("Body hit target #"):
        return f"安全轮廓触碰目标圆 #{failure.partition('#')[2]}，已停止"
    return failure


class Viewer:
    def __init__(self, scene: Scene, display: bool = True):
        configure_chinese_font()
        self.scene = scene
        self.motion = PoseController(scene)
        self.env = make_environment(scene, display)
        # IR-SIM 2.12.0 exposes its Matplotlib canvas through EnvPlot. Keep this
        # version-specific access here; the episode API never depends on it.
        self.fig, self.ax = self.env._env_plot.fig, self.env._env_plot.ax
        # Our controls own reset/pause, avoiding IR-SIM shortcuts resetting only
        # the robot while leaving task targets behind.
        if self.env.keyboard:
            for name in ("_mpl_press_cid", "_mpl_release_cid", "_mpl_enter_cid",
                         "_mpl_leave_cid", "_mpl_close_cid"):
                cid = getattr(self.env.keyboard, name, None)
                if cid is not None:
                    self.fig.canvas.mpl_disconnect(cid)
            self.env.keyboard.close()
        self.fig.canvas.manager.set_window_title("路径规划｜IR-SIM 二维仿真沙盒")
        self.fig.set_size_inches(14.2, 9.4, forward=True)
        self.fig.set_facecolor("#f1f4f9")
        self.fig.subplots_adjust(left=.075, bottom=.22, right=.71, top=.805)
        self._card([.035, .16, .70, .715])
        self._card([.755, .745, .225, .13])
        self._card([.755, .445, .225, .28])
        self._card([.755, .145, .225, .283])
        self._card([.755, .02, .225, .12])
        self._card([.035, .087, .70, .055], shadow=False)
        self._card([.035, .015, .70, .055], shadow=False, color="#eaf0f8")
        self.ax.set_facecolor("#ffffff")
        self.ax.grid(True, color="#e9eef5", linewidth=.7)
        if scene.fence is not None:
            self.ax.add_patch(Polygon(scene.fence.vertices, fill=False, edgecolor="#475569", linewidth=1.5))
        self.ax.tick_params(axis="both", colors="#8b98aa", labelsize=9, length=0, pad=8)
        for spine in self.ax.spines.values():
            spine.set_edgecolor("#e5ebf3")
        self._card([.035, .914, .029, .045], shadow=False, color=BLUE)
        self.fig.text(.0495, .936, "↗", fontsize=22, color="white", ha="center", va="center")
        self.fig.text(.078, .94, "路径规划沙盒", fontsize=23, weight=600, color=INK)
        self.fig.text(.079, .91, "摆放点位，观察运动，验证每一次消除。", fontsize=10, color=MUTED)
        self.fig.text(.969, .942, "IR-SIM  /  二维几何实验", fontsize=10, color=MUTED, ha="right")
        self.fig.text(.969, .916, "map 目标位姿   ·   仅向前行驶   ·   米制坐标", fontsize=9,
                      color=MUTED, ha="right")
        self.fig.text(.055, .842, "仿真场地", fontsize=12, weight=600, color=INK)
        self.state_badge = self.fig.text(.708, .846, "●  就绪", fontsize=10,
                                        color="#159477", ha="right")
        self.collector = Polygon(scene.vertices(scene.collector), facecolor="#10b981",
                                 edgecolor="#159477", alpha=.22, linewidth=1.5, zorder=4)
        self.ax.add_patch(self.collector)
        self.safety_outline = Polygon(scene.vertices(scene.safety_body), facecolor=(.90, .67, .25, .09),
                                      edgecolor="#c08a2f", linestyle="--",
                                      linewidth=1.5, zorder=2.5)
        self.ax.add_patch(self.safety_outline)
        self.goal_outline = Polygon(scene.vertices(scene.body), fill=False, edgecolor=BLUE,
                                    alpha=.55, linestyle="--", linewidth=1.4, zorder=3,
                                    visible=False)
        self.ax.add_patch(self.goal_outline)
        self.goal_heading, = self.ax.plot([], [], color=BLUE, marker="+", markersize=7,
                                         linewidth=1.2, alpha=.65, zorder=3)
        self.body_alarm = Polygon(scene.vertices(scene.body), facecolor="none",
                                  edgecolor="#ef4444", linewidth=3, zorder=6, visible=False)
        self.ax.add_patch(self.body_alarm)
        self.targets = TargetCircles(self.ax, facecolor="#f6b64a", edgecolor="#cf8d23",
                                      linewidth=.75, zorder=5)
        self.hit = self.ax.scatter([], [], s=180, marker="x", color="#dc2626", linewidth=3, zorder=8)
        self.path, = self.ax.plot([], [], color="#93b4ee", linewidth=1.5, zorder=2)
        self.ax.set_xlabel("横向位置 / 米", fontsize=9, color=MUTED, labelpad=10)
        self.ax.set_ylabel("纵向位置 / 米", fontsize=9, color=MUTED, labelpad=12)
        self.fig.legend(handles=[Patch(color="#2563eb", label="车身"),
                                 Patch(color="#10b981", alpha=.4, label="消除区"),
                                 Patch(facecolor="none", edgecolor="#c08a2f", linestyle="--", label="安全轮廓"),
                                 Patch(color="#f6b64a", label=f"目标 Ø{scene.target_diameter * 100:g} cm")],
                        loc="upper left", bbox_to_anchor=(.16, .862), frameon=False,
                        fontsize=9, ncol=4, handlelength=1, handleheight=.8,
                        labelcolor=MUTED, columnspacing=1.8)
        self.fig.text(.055, .048, "W / ↑  前进    A / ←  左转    D / →  右转    空格  暂停 / 继续",
                      fontsize=9, color="#637590")
        self.fig.text(.055, .028, "左键添加点  ·  右键删除点  ·  Shift + 左键设置目标  ·  R 重置  ·  Esc 停止",
                      fontsize=9, color="#637590")
        self.status = self.fig.text(.055, .109, "", fontsize=10, color=INK)
        self.message = self.fig.text(.775, .068, "准备就绪，请选择动作。", fontsize=9,
                                     color="#52627b", va="top", linespacing=1.4)
        self.widgets = []
        self.pose_boxes: dict[str, TextBox] = {}
        self.dimension_boxes: dict[str, TextBox] = {}
        self._rendered_dimensions = None
        self._controls()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.sync()

    @property
    def command(self):
        return self.motion.command

    @property
    def paused(self):
        return self.motion.paused

    @paused.setter
    def paused(self, value):
        self.motion.paused = value

    def _card(self, rect, *, shadow=True, color="white"):
        x, y, width, height = rect
        if shadow:
            self.fig.add_artist(FancyBboxPatch(
                (x, y - .003), width, height, boxstyle="round,pad=0,rounding_size=0.012",
                transform=self.fig.transFigure, facecolor="#e3e9f2", edgecolor="none", zorder=-3))
        self.fig.add_artist(FancyBboxPatch(
            (x, y), width, height, boxstyle="round,pad=0,rounding_size=0.012",
            transform=self.fig.transFigure, facecolor=color, edgecolor="none", zorder=-2))

    def _button(self, rect, label, callback, *, primary=False):
        button = Button(self.fig.add_axes(rect), label,
                        color=BLUE if primary else "#eef3fc",
                        hovercolor="#2854c9" if primary else "#dfe9fb")
        for spine in button.ax.spines.values():
            spine.set_visible(False)
        button.label.set_color("white" if primary else "#3b5685")
        button.label.set_fontsize(10)
        button.label.set_fontweight(500)
        button.on_clicked(callback)
        self.widgets.append(button)
        return button

    def _field(self, rect, label, initial):
        box = TextBox(self.fig.add_axes(rect), label + "  ", initial=initial,
                      color="#f5f7fb", hovercolor="#edf2fa")
        for spine in box.ax.spines.values():
            spine.set_color("#e2e8f2")
            spine.set_linewidth(.8)
        box.label.set_color("#64748b")
        box.label.set_fontsize(10)
        box.text_disp.set_color(INK)
        box.text_disp.set_fontsize(11)
        return box

    def _controls(self):
        self.fig.text(.775, .847, "场景选择", fontsize=11, weight=600, color=INK)
        radio = RadioButtons(self.fig.add_axes([.77, .755, .19, .08], facecolor="white"),
                             ("前方点群", "四周包围", "随机分布"),
                             active=("front", "surround", "random").index(self.scene.scenario),
                             activecolor=BLUE)
        for spine in radio.ax.spines.values():
            spine.set_visible(False)
        for label in radio.labels:
            label.set_color("#536780")
            label.set_fontsize(10)
        radio.on_clicked(lambda label: self.reset({"前方点群": "front", "四周包围": "surround",
                                                  "随机分布": "random"}[label]))
        self.widgets.append(radio)
        self.scenario_radio = radio
        self.fig.text(.775, .697, "尺寸设置", fontsize=11, weight=600, color=INK)
        self.fig.text(.958, .698, "单位：米", fontsize=8, color=MUTED, ha="right")
        for key, label, y in (("body_length", "车身长度", .655),
                              ("body_width", "车身宽度", .616),
                              ("collector_width", "消除区宽度", .577),
                              ("collector_depth", "消除区深度", .538),
                              ("body_margin", "车身外扩", .499)):
            box = self._field([.885, y, .075, .027], label, f"{self.scene.dimensions[key]:g}")
            self.dimension_boxes[key] = box
            self.widgets.append(box)
        self._button([.775, .455, .185, .032], "应用尺寸 · 停止当前动作", lambda _: self.apply_dimensions(), primary=True)
        self.fig.text(.775, .401, "目标位姿", fontsize=11, weight=600, color=INK)
        self.fig.text(.958, .402, "map 坐标系", fontsize=8, color=MUTED, ha="right")
        for key, label, value, y in (("x", "目标 x / 米", self.scene.pose[0], .36),
                                      ("y", "目标 y / 米", self.scene.pose[1], .322),
                                      ("yaw", "目标 yaw / °", math.degrees(self.scene.pose[2]), .284)):
            box = self._field([.875, y, .085, .027], label, f"{value:g}")
            self.pose_boxes[key] = box
            self.widgets.append(box)
        self.heading_box = self.pose_boxes["yaw"]
        self._button([.775, .237, .185, .032], "前往目标位姿", lambda _: self.send_goal(), primary=True)
        self._button([.775, .194, .185, .03], "↑  向前 0.25 米", lambda _: self.start("move", .25))
        self.pause_button = self._button([.775, .158, .0875, .026], "暂停", lambda _: self.toggle_pause())
        self._button([.8725, .158, .0875, .026], "重置", lambda _: self.reset())
        self._button([.775, .089, .185, .033], "保存截图和状态  ↗", lambda _: self.save())

    def note(self, message: str):
        self.message.set_text("\n".join(textwrap.fill(line, width=21)
                                         for line in message.splitlines()))

    def apply_dimensions(self):
        try:
            dimensions = {key: finite(box.text) for key, box in self.dimension_boxes.items()}
            self.scene.set_dimensions(**dimensions)
        except ValueError as error:
            self.note(error_text(error))
            return
        self.motion.stop()
        self.pause_button.label.set_text("暂停")
        self.note("尺寸已应用，当前动作已停止。")
        self.sync()

    def start(self, kind: str, amount: float):
        x, y, yaw = self.scene.pose
        try:
            amount = finite(amount)
            if kind == "move":
                if amount < 0:
                    raise ValueError("Reverse motion is disabled")
                self.set_goal(x + amount * math.cos(yaw), y + amount * math.sin(yaw), yaw)
            else:
                self.set_goal(x, y, yaw + amount)
        except ValueError as error:
            self.note(error_text(error))

    def send_goal(self):
        try:
            x, y, yaw = (finite(self.pose_boxes[key].text) for key in ("x", "y", "yaw"))
            self.set_goal(x, y, math.radians(yaw))
        except ValueError as error:
            self.note(error_text(error))

    def set_goal(self, x, y, yaw):
        self.motion.set_goal(x, y, yaw)
        self._update_pose_fields(*self.motion.goal)
        self.pause_button.label.set_text("暂停")
        self.note("目标已接收：先转向并前进，\n到达后调整目标朝向。")

    def _update_pose_fields(self, x, y, yaw):
        for key, value in (("x", x), ("y", y), ("yaw", math.degrees(yaw) % 360)):
            box = self.pose_boxes[key]
            box.set_val(f"{value:.3f}".rstrip("0").rstrip("."))

    def turn_relative(self, degrees: float):
        self.start("turn", math.radians(degrees))

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.label.set_text("继续" if self.paused else "暂停")

    def reset(self, scenario=None):
        try:
            self.scene.reset(scenario)
        except ValueError as error:
            self.scenario_radio.eventson = False
            try:
                self.scenario_radio.set_active(("front", "surround", "random").index(self.scene.scenario))
            finally:
                self.scenario_radio.eventson = True
            self.note(error_text(error))
            return
        self.motion.stop()
        self.pause_button.label.set_text("暂停")
        for key, box in self.dimension_boxes.items():
            box.set_val(f"{self.scene.dimensions[key]:g}")
        self._update_pose_fields(*self.scene.pose)
        self.note("场景已重置，准备就绪。")
        self.sync()

    def on_key(self, event):
        if any(box.capturekeystrokes for box in (*self.pose_boxes.values(), *self.dimension_boxes.values())):
            return
        if event.key in ("w", "up"):
            self.start("move", .25)
        elif event.key in ("a", "left"):
            self.turn_relative(15)
        elif event.key in ("d", "right"):
            self.turn_relative(-15)
        elif event.key == " ":
            self.toggle_pause()
        elif event.key == "r":
            self.reset()
        elif event.key == "escape":
            self.motion.stop()
            self.note("已停止当前动作。")

    def on_click(self, event):
        toolbar = self.fig.canvas.toolbar
        if event.inaxes is not self.ax or (toolbar and toolbar.mode):
            return
        if event.button == MouseButton.LEFT and event.key == "shift":
            try:
                self.set_goal(event.xdata, event.ydata, math.radians(finite(self.heading_box.text)))
            except ValueError as error:
                self.note(error_text(error))
            return
        if self.command:
            self.note("请先按 Esc 停止动作，再编辑目标点。")
            return
        if self.scene.failure:
            self.note("请先重置场景，再编辑目标点。")
            return
        try:
            if event.button == MouseButton.LEFT:
                self.scene.add_target(event.xdata, event.ydata)
            elif event.button == MouseButton.RIGHT:
                self.scene.remove_nearest(event.xdata, event.ydata)
            self.sync()
        except ValueError as error:
            self.note(error_text(error))

    def tick(self):
        was_active = self.command is not None
        self.motion.step()
        if was_active and self.command is None:
            self.note("安全轮廓碰撞，请重置后重试。" if self.scene.failure else "已到达目标位姿。")
        self.sync()

    def sync(self):
        dimensions = (self.scene.dimensions["body_length"], self.scene.dimensions["body_width"])
        if dimensions != self._rendered_dimensions:
            robot = self.env.robot
            robot.plot_clear(all=True)
            robot.gf = GeometryFactory.create_geometry(name="rectangle", length=dimensions[0], width=dimensions[1])
            robot._init_geometry()
            robot.G, robot.h, robot.cone_type, robot.convex_flag = robot.gf.get_init_Gh()
            robot.info.G, robot.info.h = robot.G, robot.h
            robot._init_plot(self.ax)
            self._rendered_dimensions = dimensions
        self.env.robot.set_state(list(self.scene.pose))
        self.env.step()  # external mode: synchronize IR-SIM with the episode pose
        self.collector.set_xy(self.scene.vertices(self.scene.collector))
        self.safety_outline.set_xy(self.scene.vertices(self.scene.safety_body))
        self.safety_outline.set_edgecolor("#dc5260" if self.scene.failure else "#c08a2f")
        self.goal_outline.set_visible(self.motion.goal is not None)
        if self.motion.goal:
            gx, gy, yaw = self.motion.goal
            c, s = math.cos(yaw), math.sin(yaw)
            self.goal_outline.set_xy([(gx + c * x - s * y, gy + s * x + c * y)
                                      for x, y in self.scene.body.corners])
            length = self.scene.body.xmax
            self.goal_heading.set_data([gx, gx + length * c], [gy, gy + length * s])
        else:
            self.goal_heading.set_data([], [])
        self.body_alarm.set_xy(self.scene.vertices(self.scene.body))
        self.body_alarm.set_visible(bool(self.scene.failure))
        self.targets.set_targets(self.scene.targets.values(), self.scene.target_radius)
        self.hit.set_offsets(np.array([self.scene.collision_point] if self.scene.collision_point else []).reshape(-1, 2))
        path = np.array(self.scene.path)
        self.path.set_data(path[:, 0], path[:, 1])
        x, y, theta = self.scene.pose
        self.status.set_text(f"剩余 {len(self.scene.targets)}  |  已消除 {len(self.scene.collected)}  |  "
                             f"位置 ({x:.2f}, {y:.2f}) 米  |  朝向 {math.degrees(theta) % 360:.1f}°  |  "
                             f"随机种子 {self.scene.seed}")
        state = ("发生碰撞" if self.scene.failure else "已暂停" if self.paused else
                 "行驶中" if self.command and self.command[0] == "move" else
                 "转向中" if self.command else "已到达" if self.motion.goal else "就绪")
        self.state_badge.set_text(f"●  {state}")
        self.state_badge.set_color("#dc5260" if self.scene.failure else "#b28129" if self.paused else "#159477")
        self.ax.set_title(failure_text(self.scene.failure) or "", fontsize=10,
                          color="#dc5260", pad=10)

    def save(self, path: Path | None = None):
        path = path or OUTPUT / f"scene-{datetime.now():%Y%m%d-%H%M%S-%f}.png"
        path = path.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.env.render(.001)
        self.fig.savefig(path, dpi=120, facecolor=self.fig.get_facecolor())
        payload = self.scene.snapshot() | {"config": self.scene.config,
                                           "goal": self.motion.goal, "paused": self.paused}
        path.with_suffix(".json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self.note("已保存截图和状态文件，\n位置：sim/output/。" if path.parent == OUTPUT.resolve()
                  else f"已保存：{path.name}\n及同名状态文件。")
        return path

    def run(self):
        try:
            while plt.fignum_exists(self.fig.number):
                self.tick()
                self.env.render(.025)
        except KeyboardInterrupt:
            pass
        finally:
            # Closing a window must not create a new blank window via plt.pause.
            self.env.display = False
            self.env.end()


def main():
    parser = argparse.ArgumentParser(description="独立的 IR-SIM 目标点消除仿真场景")
    parser.add_argument("--scenario", choices=("front", "surround", "random"), default="front")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--robot-config", type=Path, help="指定车辆配置文件，默认读取 sim/config/robot.yaml")
    parser.add_argument("--snapshot", type=Path, help="保存截图和状态文件，不打开窗口")
    parser.add_argument("--heading", type=float, help="启动时先转到指定绝对角度，单位为度")
    parser.add_argument("--distance", type=float, help="启动时向前移动指定距离，单位为米")
    args = parser.parse_args()
    scene = Scene(args.config, args.scenario, args.seed, robot_config=args.robot_config)
    if args.heading is not None:
        scene.turn_to(args.heading)
    if args.distance is not None:
        scene.forward(args.distance)
    viewer = Viewer(scene, display=args.snapshot is None)
    if args.snapshot:
        try:
            print(viewer.save(args.snapshot))
        finally:
            viewer.env.end()
    else:
        viewer.run()


if __name__ == "__main__":
    main()
