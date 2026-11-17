"""Run the existing MuJoCo arena and its GPU renderer on the Mac."""

import ctypes
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from math import atan2, isfinite
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic
from urllib.parse import parse_qs, urlsplit

import mujoco
import numpy as np
import yaml

from disc_scatter import DiscScatter
from elimination_overlay import EliminationOverlay


ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 1000, 700
WHEELS = ("left_wheel_joint", "right_wheel_joint")
BRIDGE_PORT = 8765
FRAME_PORT = 8767
WEB_ORIGIN = f"http://127.0.0.1:{BRIDGE_PORT}"
MAX_OVERLAY_BYTES = 1024 * 1024
SIM_TIMEOUT = 0.25
DEFAULT_VIEW = (90.0, -90.0, 1.0, 0.0, 0.0)


class Latest:
    def __init__(self):
        self.lock = Lock()
        self.state = None
        self.wheels = (0.0, 0.0)
        self.wheels_at = -float("inf")
        self.frame = b""
        self.frame_id = 0
        self.viewer_at = -float("inf")
        self.camera_view = DEFAULT_VIEW
        self.elimination_overlay = None
        self.scatter = None
        self.discs = ()

    def exchange(self, exit_signal):
        connection = HTTPConnection("127.0.0.1", BRIDGE_PORT, timeout=0.15)
        try:
            while not exit_signal.wait(0.018):
                with self.lock:
                    state = self.state
                if state is None:
                    continue
                try:
                    connection.request("POST", "/api/sim", json.dumps(state),
                                       {"Content-Type": "application/json"})
                    response = connection.getresponse()
                    wheels = json.loads(response.read())
                    if response.status != 200 or len(wheels) != 2:
                        raise ValueError("Invalid ROS 2 wheel command")
                    with self.lock:
                        self.wheels = wheels
                        self.wheels_at = monotonic()
                except (OSError, ValueError, TypeError):
                    connection.close()
        finally:
            connection.close()


def parse_camera_view(query):
    values = tuple(map(float, query.split(",")))
    if len(values) != len(DEFAULT_VIEW) or not all(map(isfinite, values)):
        raise ValueError("Expected azimuth, elevation, zoom, x and y")
    azimuth, elevation, zoom, x, y = values
    return (
        azimuth % 360,
        min(-20, max(-90, elevation)),
        min(2.5, max(1, zoom)),
        min(2.5, max(-2.5, x)),
        min(1.8, max(-1.8, y)),
    )


def set_camera_view(camera, view):
    azimuth, elevation, zoom, x, y = view
    camera.azimuth = azimuth
    camera.elevation = elevation
    camera.distance = 6.9 / zoom
    camera.lookat[:] = (x, y, 0.2)


class FrameHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    timeout = 2.0

    def do_GET(self):
        url = urlsplit(self.path)
        latest = self.server.latest
        if url.path != "/frame" or self.headers.get("Origin") != WEB_ORIGIN:
            self.send_error(404)
            return
        query = parse_qs(url.query)
        after = query.get("after", [""])[0]
        try:
            view = parse_camera_view(query["view"][0]) if "view" in query else None
        except ValueError:
            self.send_error(400, "Invalid camera view")
            return
        with latest.lock:
            latest.viewer_at = monotonic()
            if view is not None:
                latest.camera_view = view
            sequence = str(latest.frame_id)
            pixels = latest.frame if sequence != after else b""

        self.send_response(200 if pixels else 204)
        self.send_header("Access-Control-Allow-Origin", WEB_ORIGIN)
        self.send_header("Access-Control-Expose-Headers", "X-Frame, X-Width, X-Height")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(pixels)))
        self.send_header("X-Frame", sequence)
        self.send_header("X-Width", str(WIDTH))
        self.send_header("X-Height", str(HEIGHT))
        self.end_headers()
        if pixels:
            self.wfile.write(pixels)

    def do_OPTIONS(self):
        if self.path not in ("/overlay", "/scatter") or self.headers.get("Origin") != WEB_ORIGIN:
            self.send_error(404)
            return
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", WEB_ORIGIN)
        self.send_header("Access-Control-Allow-Methods", "POST")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self):
        if self.path not in ("/overlay", "/scatter") or self.headers.get("Origin") != WEB_ORIGIN:
            self.close_connection = True
            self.send_error(404)
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= MAX_OVERLAY_BYTES:
                raise ValueError("请发送 1 MB 以内的 JSON 数据。")
            document = json.loads(self.rfile.read(size))
            if self.path == "/overlay":
                overlay = EliminationOverlay(document)
                with self.server.latest.lock:
                    self.server.latest.elimination_overlay = overlay
            else:
                self.scatter_discs(document)
        except (UnicodeError, json.JSONDecodeError):
            self.reply_text(400, "文件无法解析为 JSON。")
            return
        except (ValueError, OverflowError, OSError) as exc:
            self.close_connection = True
            self.reply_text(400, str(exc))
            return

        self.reply_text(200)

    def scatter_discs(self, document):
        latest = self.server.latest
        with latest.lock:
            scatter, pose, overlay = latest.scatter, latest.state, latest.elimination_overlay
        if scatter is None:
            raise ValueError("正在等待场景初始化。")
        count = document.get("count") if isinstance(document, dict) else None
        discs = scatter.place(count, pose, overlay, np.random.default_rng())
        with latest.lock:
            if overlay is not latest.elimination_overlay:
                raise ValueError("消除区刚刚更新，请重新撒布。")
            latest.discs = discs

    def reply_text(self, status, message=""):
        body = message.encode()
        self.send_response(status)
        self.send_header("Access-Control-Allow-Origin", WEB_ORIGIN)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_request(self, code="-", size="-"):
        pass


def run_simulation(latest, exit_signal):
    model = mujoco.MjModel.from_xml_path(str(ROOT / "scenes" / "barrier_arena.xml"))
    data = mujoco.MjData(model)
    base_id = model.joint("base_free").id
    base_body = model.jnt_bodyid[base_id]
    base_start = model.jnt_qposadr[base_id]
    wheel_joints = [model.joint(name).id for name in WHEELS]
    wheel_positions = model.jnt_qposadr[wheel_joints]
    wheel_velocities = model.jnt_dofadr[wheel_joints]
    actuators = [model.actuator(name).id for name in WHEELS]
    ticks_per_step = max(1, round(0.01 / model.opt.timestep))
    interval = ticks_per_step * model.opt.timestep
    body_velocity = np.empty(6)
    mujoco.mj_forward(model, data)
    config_path = ROOT.parent / "sim" / "config" / "robot.yaml"
    robot = yaml.safe_load(config_path.read_text())
    scatter = DiscScatter(model, data, robot)
    default_overlay = EliminationOverlay({"rings": [{"points_m": robot["collector"]["polygon"]}]})
    with latest.lock:
        if latest.elimination_overlay is None:
            latest.elimination_overlay = default_overlay
        latest.scatter = scatter

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    model.vis.global_.fovy = model.cam_fovy[model.camera("arena_overview").id]
    set_camera_view(camera, DEFAULT_VIEW)
    with mujoco.Renderer(model, width=WIDTH, height=HEIGHT) as renderer:
        renderer.update_scene(data, camera=camera)
        renderer.render()
        gl_string = ctypes.CDLL("/System/Library/Frameworks/OpenGL.framework/OpenGL").glGetString
        gl_string.argtypes = [ctypes.c_uint]
        gl_string.restype = ctypes.c_char_p
        print(f"MuJoCo OpenGL renderer: {gl_string(0x1F01).decode()}", flush=True)
        print(f"Live frame service listening on 127.0.0.1:{FRAME_PORT}", flush=True)
        last_rendered = None
        rendered_view = None
        rendered_overlay = None
        rendered_discs = None
        frame_due = 0.0
        deadline = monotonic()

        while not exit_signal.is_set():
            now = monotonic()
            with latest.lock:
                connected = now - latest.wheels_at < SIM_TIMEOUT
                wheels = latest.wheels if connected else (0.0, 0.0)
                has_viewer = now - latest.viewer_at < 1.0
                view = latest.camera_view
                overlay = latest.elimination_overlay
                discs = latest.discs
            data.ctrl[actuators] = wheels
            mujoco.mj_step(model, data, nstep=ticks_per_step)
            base_pose = data.qpos[base_start:base_start + 7]
            angles = data.qpos[wheel_positions]
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, base_body, body_velocity, 1)
            state = {
                "time": data.time,
                "position": base_pose[:3].tolist(),
                "rotation": base_pose[3:].tolist(),
                "body_velocity": body_velocity.tolist(),
                "wheel_position": angles.tolist(),
                "wheel_velocity": data.qvel[wheel_velocities].tolist(),
            }
            with latest.lock:
                latest.state = state

            if has_viewer and now >= frame_due:
                visible_state = np.concatenate((base_pose, angles))
                moved = last_rendered is None or np.max(np.abs(last_rendered - visible_state)) >= 0.0007
                if moved or view != rendered_view or overlay is not rendered_overlay or discs is not rendered_discs:
                    set_camera_view(camera, view)
                    renderer.update_scene(data, camera=camera)
                    if overlay is not None:
                        # Use the same world pose as the frame. Roll and pitch stay out of the overlay.
                        x, y = data.xpos[base_body, :2]
                        orientation = data.xmat[base_body]
                        overlay.draw(renderer.scene, x, y, atan2(orientation[3], orientation[0]))
                    if discs:
                        scatter.draw(renderer.scene, discs)
                    pixels = renderer.render().tobytes()
                    with latest.lock:
                        latest.frame = pixels
                        latest.frame_id += 1
                    last_rendered = visible_state
                    rendered_view = view
                    rendered_overlay = overlay
                    rendered_discs = discs
                    frame_due = now + 1 / 20

            deadline += interval
            if deadline < monotonic() - 0.05:
                deadline = monotonic()
            exit_signal.wait(max(0.0, deadline - monotonic()))


def main():
    latest = Latest()
    exit_signal = Event()
    with ThreadingHTTPServer(("127.0.0.1", FRAME_PORT), FrameHandler) as server:
        server.latest = latest
        threads = [
            Thread(target=server.serve_forever, daemon=True),
            Thread(target=latest.exchange, args=(exit_signal,), daemon=True),
        ]
        for thread in threads:
            thread.start()
        try:
            run_simulation(latest, exit_signal)
        except KeyboardInterrupt:
            pass
        finally:
            exit_signal.set()
            server.shutdown()
            for thread in threads:
                thread.join()


if __name__ == "__main__":
    main()
