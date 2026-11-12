"""Export a GIF from the executed pose trace using IR-SIM's external mode."""

from collections import defaultdict
import math

from matplotlib.animation import PillowWriter
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Polygon, Wedge

from .app import configure_chinese_font, make_environment
from .cameras import to_map
from .drawing import TargetCircles

BLUE = "#2478ad"
PURPLE = "#8e55bd"
GREEN = "#148960"
AMBER = "#f6b64a"
SLATE = "#4b5563"
INK = "#24334a"


def save_replay(scene, cameras, result, output):
    configure_chinese_font()
    env = make_environment(scene, display=False)
    try:
        _write_gif(env, scene.clone(), cameras, result, output)
    finally:
        env.end()


def _write_gif(env, scene, cameras, result, output):
    fig, ax = env._env_plot.fig, env._env_plot.ax
    fig.set_size_inches(11.3, 6.2)
    fig.patch.set_facecolor("#f1f5f9")
    fig.subplots_adjust(left=.07, right=.72, bottom=.17, top=.84)
    ax.set(xlim=(0, scene.width), ylim=(0, scene.height), aspect="equal",
           xlabel="map x / 米", ylabel="map y / 米")
    ax.grid(alpha=.2)
    ax.set_axisbelow(True)
    fig.text(.065, .94, result["title"], fontsize=20, weight=600, color=INK)
    fig.text(.067, .892, "双相机主动感知 · IR-SIM", fontsize=10, color=SLATE)
    fig.text(.96, .933, f"种子 {result['seed']}", ha="right", fontsize=10, color=SLATE)
    status = fig.text(.735, .81, "", fontsize=11, va="top", color=INK, linespacing=1.65)

    sectors = {}
    for name, color in (("bottom", BLUE), ("local", PURPLE)):
        sector = Wedge((0, 0), cameras[name].max_range, 0, 0,
                       facecolor=color, edgecolor=color, alpha=.17, linewidth=1.4, zorder=1)
        ax.add_patch(sector)
        origin, = ax.plot([], [], "o", color=color, markersize=4, zorder=7)
        sectors[name] = sector, origin
    for x, y, radius in scene.obstacles:
        ax.add_patch(Circle((x, y), radius, color=SLATE, zorder=5))
    targets = TargetCircles(ax, facecolor=AMBER, edgecolor="#b87917", linewidth=.8, zorder=5)
    targets.set_targets(scene.targets.values(), scene.target_radius)
    for key, (x, y) in scene.targets.items():
        ax.annotate(f"#{key}", (x, y), xytext=(5, 5), textcoords="offset points",
                    color="#875715", fontsize=9, zorder=7)
    safety = Polygon(scene.vertices(scene.safety_body), fill=False, edgecolor="#5b697e",
                     linestyle="--", linewidth=1, zorder=4)
    collector = Polygon(scene.vertices(scene.collector), facecolor="#10b981", alpha=.55, zorder=4)
    ax.add_patch(safety)
    ax.add_patch(collector)
    path, = ax.plot([], [], color="#315fb5", linewidth=1.6, zorder=3)
    hints, = ax.plot([], [], linestyle="none", marker="x", color=BLUE, markersize=7, zorder=7)
    confirmed, = ax.plot([], [], linestyle="none", marker="o", markerfacecolor="none",
                         markeredgecolor=GREEN, markersize=10, markeredgewidth=1.6, zorder=8)
    fig.legend(handles=[
        Patch(facecolor=BLUE, alpha=.25, label="底部广角视场"),
        Patch(facecolor=PURPLE, alpha=.25, label="局部相机视场"),
        Patch(facecolor=SLATE, label="遮挡物"),
        Patch(facecolor=AMBER, label="目标真值"),
        Line2D([], [], color=BLUE, linestyle="none", marker="x", label="广角粗略 XY"),
        Line2D([], [], color=GREEN, linestyle="none", marker="o", markerfacecolor="none", label="局部确认"),
    ], loc="lower center", bbox_to_anchor=(.5, .045), ncol=6, frameon=False, fontsize=8.5)

    captures = defaultdict(list)
    for index, observation in enumerate(result["observations"]):
        captures[observation["trace_index"]].append(index)
    requests = {request["observation_index"]: request for request in result["view_requests"]}
    handoff = result["handoff"]
    trace = result["trace"]
    writer = PillowWriter(fps=8)
    active_observation = 0
    traversed = []

    def draw(index, observation_index):
        frame = trace[index]
        scene.pose = tuple(frame["pose"])
        env.robot.set_state(list(scene.pose))
        env.step()
        env.render()
        safety.set_xy(scene.vertices(scene.safety_body))
        collector.set_xy(scene.vertices(scene.collector))
        path.set_data(*zip(*traversed))
        for name, (sector, origin) in sectors.items():
            x, y, yaw = cameras[name].map_pose(scene.pose)
            half = math.degrees(cameras[name].horizontal_fov) / 2
            sector.set_center((x, y))
            sector.set_theta1(math.degrees(yaw) - half)
            sector.set_theta2(math.degrees(yaw) + half)
            origin.set_data([x], [y])
        observation = result["observations"][observation_index]
        bottom, local = observation["bottom"], observation["local"]
        for artist, camera_frame in ((hints, bottom), (confirmed, local)):
            points = [to_map(camera_frame["camera_pose_map"], item["xy_m"])
                      for item in camera_frame["detections"]]
            artist.set_data(*zip(*points) if points else ([], []))
        phase = {"initial": "初始观察", "sdk_view": "执行 SDK 观察转向",
                 "preset_viewpoint": "前往预设观察位姿"}[frame["stage"]]
        request = requests.get(observation_index)
        lines = [phase, f"仿真时间 {frame['timestamp_s']:.2f} 秒", "",
                 f"最近一帧  {local['timestamp_s']:.2f} 秒",
                 f"底部线索  {len(bottom['detections'])} 条",
                 f"局部坐标  {len(local['detections'])} 个"]
        if request and request["selected_index"] is not None:
            view = request["suggestions"][request["selected_index"]]
            lines.extend(["", f"SDK 建议转角 {math.degrees(view['yaw_offset']):+.1f}°"])
        if not local["detections"]:
            reasons = set(observation["truth_visibility"]["local"].values())
            if "occluded" in reasons:
                lines.append("有目标被遮挡")
            if "out_of_range" in reasons:
                lines.append("有目标超出局部有效距离")
        if handoff and observation_index == handoff["observation_index"]:
            lines.extend(["", "已交接给拾取层", *(
                f"map  ({x:.2f}, {y:.2f}) 米" for x, y in handoff["targets_xy_m"]
            )])
        elif index == len(trace) - 1 and result["final"]["failure"]:
            lines.extend(["", "运动已因碰撞停止"])
        status.set_text("\n".join(lines))

    with writer.saving(fig, str(output), dpi=88):
        for index, frame in enumerate(trace):
            traversed.append(tuple(frame["pose"][:2]))
            if captures[index]:
                for observation_index in captures[index]:
                    active_observation = observation_index
                    draw(index, active_observation)
                    for _ in range(8):
                        writer.grab_frame()
            elif index % 3 == 0 or index == len(trace) - 1:
                draw(index, active_observation)
                writer.grab_frame()
