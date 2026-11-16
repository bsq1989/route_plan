"""Expose the native MuJoCo process and browser controls to ROS 2 Jazzy."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
from pathlib import Path
from threading import Lock, Thread
from time import monotonic

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


WHEELS = ["left_wheel_joint", "right_wheel_joint"]
COMMANDS = {
    "forward": (0.25, 0.0),
    "reverse": (-0.15, 0.0),
    "left": (0.0, 0.6),
    "right": (0.0, -0.6),
    "stop": (0.0, 0.0),
}
COMMAND_LEASE = 0.3
SIM_TIMEOUT = 0.25
WEB_ROOT = Path(__file__).resolve().parents[1] / "output" / "arena-web" / "client"
STATE_FIELDS = {"position": 3, "rotation": 4, "body_velocity": 6, "wheel_position": 2, "wheel_velocity": 2}


class ArenaBridge(Node):
    def __init__(self):
        super().__init__("arena_web")
        self.lock = Lock()
        self.pose = None
        self.state_at = -math.inf
        self.wheel_command = [0.0, 0.0]
        self.wheels_at = -math.inf
        self.command = "stop"
        self.command_until = 0.0
        self.was_driving = False
        self.drive = self.create_publisher(TwistStamped, "/sim/cmd_vel", 10)
        self.clock = self.create_publisher(Clock, "/clock", 1)
        self.feedback = self.create_publisher(JointState, "/sim/wheel_feedback", 1)
        self.truth = self.create_publisher(Odometry, "/sim/ground_truth/odom", 1)
        self.create_subscription(JointState, "/sim/wheel_commands", self.on_wheels, 1)
        self.create_timer(0.05, self.publish_command)

    def on_wheels(self, message):
        try:
            command = [message.velocity[message.name.index(name)] for name in WHEELS]
        except (ValueError, IndexError):
            return
        if all(math.isfinite(value) for value in command):
            with self.lock:
                self.wheel_command = command
                self.wheels_at = monotonic()

    def exchange_state(self, state):
        nanoseconds = round(state["time"] * 1e9)
        stamp = Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)
        self.clock.publish(Clock(clock=stamp))

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = WHEELS
        joints.position = state["wheel_position"]
        joints.velocity = state["wheel_velocity"]
        self.feedback.publish(joints)

        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = "world"
        truth.child_frame_id = "base_link"
        x, y, z = state["position"]
        w, qx, qy, qz = state["rotation"]
        truth.pose.pose.position.x, truth.pose.pose.position.y, truth.pose.pose.position.z = x, y, z
        truth.pose.pose.orientation.w = w
        truth.pose.pose.orientation.x = qx
        truth.pose.pose.orientation.y = qy
        truth.pose.pose.orientation.z = qz
        wx, wy, wz, vx, vy, vz = state["body_velocity"]
        truth.twist.twist.linear.x, truth.twist.twist.linear.y, truth.twist.twist.linear.z = vx, vy, vz
        truth.twist.twist.angular.x, truth.twist.twist.angular.y, truth.twist.twist.angular.z = wx, wy, wz
        self.truth.publish(truth)

        yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
        with self.lock:
            now = monotonic()
            self.pose = {"x": x, "y": y, "yaw": math.degrees(yaw)}
            self.state_at = now
            return self.wheel_command if now - self.wheels_at < SIM_TIMEOUT else [0.0, 0.0]

    def ready(self, now):
        return now - self.state_at < SIM_TIMEOUT and now - self.wheels_at < SIM_TIMEOUT

    def status(self):
        with self.lock:
            online = self.ready(monotonic())
            return {"online": online, "pose": self.pose if online else None}

    def set_command(self, command):
        with self.lock:
            now = monotonic()
            if command != "stop" and not self.ready(now):
                return False
            self.command = command
            self.command_until = now + COMMAND_LEASE
            return True

    def publish_command(self):
        with self.lock:
            now = monotonic()
            driving = self.command != "stop" and now < self.command_until and self.ready(now)
            if driving or self.was_driving:
                message = TwistStamped()
                # A zero stamp lets the controller use its current simulation time.
                message.twist.linear.x, message.twist.angular.z = COMMANDS[self.command if driving else "stop"]
                self.drive.publish(message)
            self.was_driving = driving


class WebHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    timeout = 3.0

    def reply(self, status, content_type="text/plain", body=b""):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def json_reply(self, value):
        self.reply(200, "application/json", json.dumps(value).encode())

    def do_GET(self):
        if self.path == "/api/status":
            self.json_reply(self.server.bridge.status())
            return
        path = (WEB_ROOT / self.path.lstrip("/")).resolve() if self.path != "/" else WEB_ROOT / "index.html"
        if path.is_relative_to(WEB_ROOT) and path.is_file():
            self.reply(200, mimetypes.guess_type(path)[0] or "application/octet-stream", path.read_bytes())
        else:
            self.reply(404)

    def do_POST(self):
        if self.headers.get_content_type() != "application/json":
            self.close_connection = True
            self.reply(400)
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 1024:
                raise ValueError
            value = json.loads(self.rfile.read(size))
            if self.path == "/api/drive" and isinstance(value, str) and value in COMMANDS:
                self.reply(204 if self.server.bridge.set_command(value) else 503)
                return
            if self.path == "/api/sim" and valid_state(value):
                self.json_reply(self.server.bridge.exchange_state(value))
                return
        except (ValueError, OSError, OverflowError):
            pass
        self.close_connection = True
        self.reply(400)

    def log_request(self, code="-", size="-"):
        pass


def valid_state(state):
    if not isinstance(state, dict):
        return False
    timestamp = state.get("time")
    return (
        isinstance(timestamp, (int, float)) and 0 <= timestamp < 1e9
        and all(
            isinstance(state.get(key), list) and len(state[key]) == size
            and all(isinstance(value, (int, float)) and math.isfinite(value) for value in state[key])
            for key, size in STATE_FIELDS.items()
        )
    )


def main():
    if not (WEB_ROOT / "index.html").is_file():
        raise SystemExit("Build the arena web client before launching web mode.")

    rclpy.init()
    bridge = ArenaBridge()
    try:
        with ThreadingHTTPServer(("0.0.0.0", 8080), WebHandler) as server:
            server.bridge = bridge
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            bridge.get_logger().info("Arena web server listening on port 8080")
            try:
                rclpy.spin(bridge)
            except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
                pass
            finally:
                server.shutdown()
                thread.join()
    finally:
        bridge.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
