"""Run the existing MuJoCo arena and its GPU renderer on the Mac."""

import ctypes
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic
from urllib.parse import parse_qs, urlsplit

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 1000, 700
WHEELS = ("left_wheel_joint", "right_wheel_joint")
BRIDGE_PORT = 8765
FRAME_PORT = 8767
SIM_TIMEOUT = 0.25


class Latest:
    def __init__(self):
        self.lock = Lock()
        self.state = None
        self.wheels = (0.0, 0.0)
        self.wheels_at = -float("inf")
        self.frame = b""
        self.frame_id = 0
        self.viewer_at = -float("inf")

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


class FrameHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    timeout = 2.0

    def do_GET(self):
        url = urlsplit(self.path)
        latest = self.server.latest
        if url.path != "/frame" or self.headers.get("Origin") != f"http://127.0.0.1:{BRIDGE_PORT}":
            self.send_error(404)
            return
        after = parse_qs(url.query).get("after", [""])[0]
        with latest.lock:
            latest.viewer_at = monotonic()
            sequence = str(latest.frame_id)
            pixels = latest.frame if sequence != after else b""

        self.send_response(200 if pixels else 204)
        self.send_header("Access-Control-Allow-Origin", f"http://127.0.0.1:{BRIDGE_PORT}")
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

    with mujoco.Renderer(model, width=WIDTH, height=HEIGHT) as renderer:
        renderer.update_scene(data, camera="arena_overview")
        renderer.render()
        gl_string = ctypes.CDLL("/System/Library/Frameworks/OpenGL.framework/OpenGL").glGetString
        gl_string.argtypes = [ctypes.c_uint]
        gl_string.restype = ctypes.c_char_p
        print(f"MuJoCo OpenGL renderer: {gl_string(0x1F01).decode()}", flush=True)
        print(f"Live frame service listening on 127.0.0.1:{FRAME_PORT}", flush=True)
        last_rendered = None
        frame_due = 0.0
        deadline = monotonic()

        while not exit_signal.is_set():
            now = monotonic()
            with latest.lock:
                connected = now - latest.wheels_at < SIM_TIMEOUT
                wheels = latest.wheels if connected else (0.0, 0.0)
                has_viewer = now - latest.viewer_at < 1.0
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
                if last_rendered is None or np.max(np.abs(last_rendered - visible_state)) >= 0.0007:
                    renderer.update_scene(data, camera="arena_overview")
                    pixels = renderer.render().tobytes()
                    with latest.lock:
                        latest.frame = pixels
                        latest.frame_id += 1
                    last_rendered = visible_state
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
