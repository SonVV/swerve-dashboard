#!/usr/bin/env python3
"""aiohttp dashboard backed by the live ROS 2 swerve simulation."""

import asyncio
import json
import math
import os
import queue
import threading
import time
from pathlib import Path

from aiohttp import WSMsgType, web
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from swerve_msgs.action import NavigateToPose
from swerve_msgs.msg import SwerveCommand
from tf2_ros import Buffer, TransformException, TransformListener


ROOT = Path(__file__).parent
WHEEL_RADIUS = float(os.getenv("WHEEL_RADIUS", "0.0315"))
MAX_LINEAR_SPEED = float(os.getenv("MAX_LINEAR_SPEED", "0.8"))
MAX_YAW_SPEED = float(os.getenv("MAX_YAW_SPEED", "2.0"))
MODULES = (
    ("FL", "Stearing1_Joint", "Wheel1_Joint"),
    ("FR", "Stearing2_Joint", "Wheel2_Joint"),
    ("RL", "Stearing4_Joint", "Wheel4_Joint"),
    ("RR", "Stearing3_Joint", "Wheel3_Joint"),
)
ACTIVE_GOAL_STATES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def clamp(value, lower, upper):
    """Limit a numeric command to the controller's configured range."""
    return max(lower, min(upper, value))


def quaternion_yaw(q):
    """Extract planar yaw from a ROS quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def encode_rle(values):
    """Compact an occupancy grid for occasional WebSocket transfer."""
    if not values:
        return []
    encoded = []
    previous = int(values[0])
    count = 1
    for raw in values[1:]:
        value = int(raw)
        if value == previous and count < 65535:
            count += 1
        else:
            encoded.extend((previous, count))
            previous, count = value, 1
    encoded.extend((previous, count))
    return encoded


class SharedState:
    """Thread-safe state shared by the ROS executor and aiohttp event loop."""

    def __init__(self):
        self.lock = threading.Lock()
        self.pose = {"x": 0.0, "y": 0.0, "heading": 0.0, "frame": "waiting"}
        self.twist = {"vx": 0.0, "vy": 0.0, "omega": 0.0}
        self.wheels = [
            {"id": module[0], "speed": 0.0, "accel": 0.0, "steering": 0.0}
            for module in MODULES
        ]
        self.command = {
            "vx": 0.0, "vy": 0.0, "omega": 0.0,
            "mode": SwerveCommand.DYNAMIC_BODY, "enabled": False, "stamp": 0.0,
        }
        self.map = None
        # Raw cells stay on the server for validating map-click navigation.
        # The browser receives the smaller RLE representation in ``self.map``.
        self.map_data = None
        self.map_revision = 0
        self.mission = "idle"
        self.publisher_active = False
        self.navigation = {
            "id": 0,
            "state": "idle",
            "message": "",
            "target": None,
            "distance_remaining": None,
        }
        self.received = {"odom": 0.0, "joints": 0.0, "map": 0.0, "tf": 0.0}

    def set_command(self, data, enabled):
        """Validate, limit and store the latest browser joystick command."""
        vx = float(data.get("vx", 0.0))
        vy = float(data.get("vy", 0.0))
        omega = float(data.get("omega", 0.0))
        mode = int(data.get("mode", SwerveCommand.DYNAMIC_BODY))
        if not all(math.isfinite(value) for value in (vx, vy, omega)):
            raise ValueError("Command contains NaN or infinity")
        if mode not in (SwerveCommand.HOLD_BODY, SwerveCommand.DYNAMIC_BODY):
            raise ValueError("Unknown drive mode")
        norm = math.hypot(vx, vy)
        if norm > MAX_LINEAR_SPEED:
            vx *= MAX_LINEAR_SPEED / norm
            vy *= MAX_LINEAR_SPEED / norm
        with self.lock:
            self.command.update(
                vx=vx if enabled else 0.0,
                vy=vy if enabled else 0.0,
                omega=clamp(omega, -MAX_YAW_SPEED, MAX_YAW_SPEED) if enabled else 0.0,
                mode=mode,
                enabled=enabled,
                stamp=time.monotonic(),
            )

    def disable(self):
        """Atomically replace the current manual command with a zero command."""
        self.set_command({}, False)

    def validate_navigation_target(self, x, y):
        """Accept only finite coordinates that lie in a known free map cell."""
        with self.lock:
            if self.map is None or self.map_data is None:
                return False, "Robot chưa scan nên chưa thể chọn tọa độ"
            model = self.map
            origin = model["origin"]
            dx, dy = x - origin["x"], y - origin["y"]
            cosine, sine = math.cos(origin["yaw"]), math.sin(origin["yaw"])
            grid_x = (cosine * dx + sine * dy) / model["resolution"]
            grid_y = (-sine * dx + cosine * dy) / model["resolution"]
            cell_x, cell_y = math.floor(grid_x), math.floor(grid_y)
            if not (0 <= cell_x < model["width"] and 0 <= cell_y < model["height"]):
                return False, "Tọa độ nằm ngoài bản đồ đã scan"
            value = self.map_data[cell_y * model["width"] + cell_x]
            if value < 0:
                return False, "Tọa độ này chưa được scan"
            if value >= 50:
                return False, "Tọa độ nằm trên vật cản"
            return True, ""

    def begin_navigation(self, x, y, yaw):
        """Reserve one browser navigation request and return its sequence id."""
        with self.lock:
            if self.navigation["state"] in ("submitting", "active"):
                return None
            request_id = self.navigation["id"] + 1
            self.navigation = {
                "id": request_id,
                "state": "submitting",
                "message": "Đang gửi mục tiêu tới ROS 2",
                "target": {"x": x, "y": y, "yaw": yaw},
                "distance_remaining": None,
            }
            return request_id

    def update_navigation(self, request_id, state, message, distance=None):
        """Update an action only if the callback still belongs to that request."""
        with self.lock:
            if self.navigation["id"] != request_id:
                return
            self.navigation["state"] = state
            self.navigation["message"] = message
            self.navigation["distance_remaining"] = distance

    def snapshot(self, include_map=False):
        """Return a JSON-safe, internally consistent telemetry snapshot."""
        now = time.monotonic()
        with self.lock:
            command_fresh = self.command["enabled"] and now - self.command["stamp"] < 0.5
            status = {key: bool(stamp and (key == "map" or now - stamp < 1.0))
                      for key, stamp in self.received.items()}
            result = {
                "type": "telemetry",
                "t": round(time.time(), 3),
                "backend": "ros2",
                "pose": dict(self.pose),
                "twist": dict(self.twist),
                "wheels": [dict(wheel) for wheel in self.wheels],
                "enabled": command_fresh,
                "drive_mode": int(self.command["mode"]),
                "mission": self.mission,
                "navigation": dict(self.navigation),
                "publisher_active": self.publisher_active,
                "status": status,
                "map_revision": self.map_revision,
            }
            if include_map and self.map is not None:
                result["map"] = dict(self.map)
            return result


class DashboardNode(Node):
    """ROS-facing bridge for telemetry, TF, manual commands and navigation."""

    def __init__(self, state):
        super().__init__("swerve_dashboard_bridge")
        self.state = state
        self.command_publisher = None
        self.stop_ticks = 0
        self.previous_speeds = [0.0] * len(MODULES)
        self.previous_acceleration = [0.0] * len(MODULES)
        self.last_joint_time = 0.0
        self.missions = {"scan": False, "navigate": False}
        # aiohttp enqueues requests; this timer consumes them on the ROS thread.
        self.navigation_requests = queue.SimpleQueue()
        self.navigate_client = ActionClient(self, NavigateToPose, "/factory/navigate")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(
            GoalStatusArray, "/factory/scan/_action/status",
            lambda message: self.on_mission("scan", message), 10)
        self.create_subscription(
            GoalStatusArray, "/factory/navigate/_action/status",
            lambda message: self.on_mission("navigate", message), 10)
        self.create_timer(0.05, self.update_pose)
        self.create_timer(0.05, self.publish_command)
        self.create_timer(0.05, self.process_navigation_requests)

    def on_odometry(self, message):
        """Publish EKF twist and use odom pose until the map TF is available."""
        pose = message.pose.pose
        twist = message.twist.twist
        if not all(math.isfinite(value) for value in (
                pose.position.x, pose.position.y, twist.linear.x,
                twist.linear.y, twist.angular.z)):
            return
        with self.state.lock:
            self.state.twist.update(vx=twist.linear.x, vy=twist.linear.y, omega=twist.angular.z)
            if self.state.pose["frame"] != "map":
                self.state.pose.update(
                    x=pose.position.x,
                    y=pose.position.y,
                    heading=math.degrees(quaternion_yaw(pose.orientation)),
                    frame=message.header.frame_id or "odom",
                )
            self.state.received["odom"] = time.monotonic()

    def on_joints(self, message):
        """Map joint names to four modules and estimate filtered acceleration."""
        positions = {name: message.position[index] for index, name in enumerate(message.name)
                     if index < len(message.position) and math.isfinite(message.position[index])}
        velocities = {name: message.velocity[index] for index, name in enumerate(message.name)
                      if index < len(message.velocity) and math.isfinite(message.velocity[index])}
        now = time.monotonic()
        dt = now - self.last_joint_time if self.last_joint_time else 0.0
        wheels = []
        for index, (module_id, steering_joint, wheel_joint) in enumerate(MODULES):
            speed = velocities.get(wheel_joint, 0.0) * WHEEL_RADIUS
            raw_acceleration = ((speed - self.previous_speeds[index]) / dt) if dt > 1e-3 else 0.0
            acceleration = 0.25 * raw_acceleration + 0.75 * self.previous_acceleration[index]
            steering = positions.get(steering_joint, 0.0)
            wheels.append({
                "id": module_id,
                "speed": round(speed, 4),
                "accel": round(acceleration, 4),
                "steering": round(math.degrees(math.atan2(math.sin(steering), math.cos(steering))), 2),
            })
            self.previous_speeds[index] = speed
            self.previous_acceleration[index] = acceleration
        self.last_joint_time = now
        with self.state.lock:
            self.state.wheels = wheels
            self.state.received["joints"] = now

    def on_map(self, message):
        """Store the occupancy grid and create its transport-friendly RLE copy."""
        expected = int(message.info.width) * int(message.info.height)
        if not message.info.width or not message.info.height or len(message.data) != expected:
            return
        origin = message.info.origin
        encoded = {
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "origin": {
                "x": float(origin.position.x),
                "y": float(origin.position.y),
                "yaw": quaternion_yaw(origin.orientation),
            },
            "rle": encode_rle(message.data),
        }
        with self.state.lock:
            self.state.map_revision += 1
            encoded["revision"] = self.state.map_revision
            self.state.map = encoded
            self.state.map_data = tuple(int(value) for value in message.data)
            self.state.received["map"] = time.monotonic()

    def on_mission(self, name, message):
        """Track action activity so autonomous and manual control cannot race."""
        self.missions[name] = any(status.status in ACTIVE_GOAL_STATES for status in message.status_list)
        with self.state.lock:
            self.state.mission = next((key for key in ("scan", "navigate") if self.missions[key]), "idle")

    def update_pose(self):
        """Prefer the localized map-frame pose over raw odometry."""
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        if not all(math.isfinite(value) for value in (translation.x, translation.y, rotation.z, rotation.w)):
            return
        with self.state.lock:
            self.state.pose.update(
                x=translation.x,
                y=translation.y,
                heading=math.degrees(quaternion_yaw(rotation)),
                frame="map",
            )
            self.state.received["tf"] = time.monotonic()

    def publish_command(self):
        """Publish fresh joystick commands and release the topic after stopping."""
        now = time.monotonic()
        with self.state.lock:
            command = dict(self.state.command)
        active = command["enabled"] and now - command["stamp"] < 0.5
        if active and self.command_publisher is None:
            self.command_publisher = self.create_publisher(
                SwerveCommand, "/swerve_controller/command", 1)
        if self.command_publisher is not None:
            message = SwerveCommand()
            if active:
                message.vx = command["vx"]
                message.vy = command["vy"]
                message.v_omega = command["omega"]
                message.mode = command["mode"]
                self.stop_ticks = 0
            else:
                message.mode = command["mode"]
                self.stop_ticks += 1
            self.command_publisher.publish(message)
            if not active and self.stop_ticks >= 12:
                self.destroy_publisher(self.command_publisher)
                self.command_publisher = None
                self.stop_ticks = 0
        with self.state.lock:
            self.state.publisher_active = self.command_publisher is not None

    def enqueue_navigation(self, request_id, x, y, yaw):
        """Called by aiohttp; SimpleQueue safely crosses the two threads."""
        self.navigation_requests.put((request_id, x, y, yaw))

    def process_navigation_requests(self):
        """Create ROS action goals only from the ROS executor thread."""
        try:
            request_id, x, y, yaw = self.navigation_requests.get_nowait()
        except queue.Empty:
            return
        if not self.navigate_client.server_is_ready():
            self.state.update_navigation(
                request_id, "failed", "Navigation action server chưa sẵn sàng")
            return
        goal = NavigateToPose.Goal()
        goal.target.header.frame_id = "map"
        goal.target.header.stamp = self.get_clock().now().to_msg()
        goal.target.pose.position.x = x
        goal.target.pose.position.y = y
        goal.target.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target.pose.orientation.w = math.cos(yaw / 2.0)
        goal.timeout_sec = 180.0
        future = self.navigate_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self.on_navigation_feedback(
                request_id, feedback.feedback),
        )
        future.add_done_callback(
            lambda response: self.on_navigation_goal_response(request_id, response))

    def on_navigation_feedback(self, request_id, feedback):
        """Expose distance-to-goal without creating a new browser event."""
        self.state.update_navigation(
            request_id,
            "active",
            feedback.phase,
            float(feedback.distance_remaining),
        )

    def on_navigation_goal_response(self, request_id, future):
        """Handle action acceptance and register the final-result callback."""
        try:
            goal_handle = future.result()
        except Exception as error:
            self.state.update_navigation(request_id, "failed", str(error))
            return
        if not goal_handle.accepted:
            self.state.update_navigation(
                request_id, "failed", "Robot từ chối mục tiêu điều hướng")
            return
        self.state.update_navigation(request_id, "active", "Đang đi tới mục tiêu")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.on_navigation_result(request_id, result))

    def on_navigation_result(self, request_id, future):
        """Convert the ROS action result to a small browser notification state."""
        try:
            result = future.result().result
            state = "succeeded" if result.success else "failed"
            message = result.message or (
                "Robot đã đến mục tiêu" if result.success else "Robot không thể đến mục tiêu")
            self.state.update_navigation(request_id, state, message, 0.0 if result.success else None)
        except Exception as error:
            self.state.update_navigation(request_id, "failed", str(error))


class RosRuntime:
    """Run rclpy in its own thread beside aiohttp's asyncio event loop."""

    def __init__(self, state):
        rclpy.init()
        self.node = DashboardNode(state)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.spin, name="ros2-dashboard", daemon=True)
        self.thread.start()

    def spin(self):
        """Block the worker thread while the ROS executor processes callbacks."""
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except Exception:
            # SIGINT can shut the global ROS context down before aiohttp enters
            # its cleanup context. Suppress only that expected shutdown race.
            if rclpy.ok():
                raise

    def shutdown(self):
        """Stop callbacks and release the ROS context during aiohttp cleanup."""
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def create_app():
    """Build the HTTP/WebSocket application and own one ROS runtime."""
    state = SharedState()
    clients = {}
    control = {"owner": None}
    runtime_holder = {"runtime": None}
    app = web.Application()
    app["state"] = state
    app["clients"] = clients

    async def index(_request):
        """Serve the single-page dashboard shell."""
        return web.FileResponse(ROOT / "index.html")

    async def health(_request):
        """Small endpoint used by operators and smoke tests."""
        snapshot = state.snapshot()
        return web.json_response({
            "status": "ok",
            "backend": "ros2",
            "clients": len(clients),
            "mission": snapshot["mission"],
            "topics": snapshot["status"],
            "manual_publisher": snapshot["publisher_active"],
            "navigation": snapshot["navigation"],
        })

    async def websocket(request):
        """Handle manual commands and click-to-navigate requests from a browser."""
        socket = web.WebSocketResponse(heartbeat=10, max_msg_size=16384)
        await socket.prepare(request)
        clients[socket] = -1
        try:
            async for message in socket:
                if message.type != WSMsgType.TEXT:
                    continue
                data = {}
                try:
                    data = json.loads(message.data)
                    packet_type = data.get("type")
                    if packet_type == "navigate":
                        x = float(data["x"])
                        y = float(data["y"])
                        yaw = float(data.get("yaw", 0.0))
                        if not all(math.isfinite(value) for value in (x, y, yaw)):
                            raise ValueError("Tọa độ chứa NaN hoặc infinity")

                        snapshot = state.snapshot()
                        if snapshot["publisher_active"]:
                            await socket.send_json({
                                "type": "navigation", "accepted": False,
                                "message": "Hãy STOP joystick và chờ publisher manual được nhả",
                            })
                            continue
                        if snapshot["mission"] != "idle":
                            await socket.send_json({
                                "type": "navigation", "accepted": False,
                                "message": f"Action {snapshot['mission']} đang chạy",
                            })
                            continue
                        valid, reason = state.validate_navigation_target(x, y)
                        if not valid:
                            await socket.send_json({
                                "type": "navigation", "accepted": False, "message": reason,
                            })
                            continue
                        request_id = state.begin_navigation(x, y, yaw)
                        if request_id is None:
                            await socket.send_json({
                                "type": "navigation", "accepted": False,
                                "message": "Một yêu cầu điều hướng đang được xử lý",
                            })
                            continue
                        runtime = runtime_holder["runtime"]
                        if runtime is None:
                            state.update_navigation(
                                request_id, "failed", "ROS 2 bridge chưa sẵn sàng")
                            await socket.send_json({
                                "type": "navigation", "accepted": False,
                                "message": "ROS 2 bridge chưa sẵn sàng",
                            })
                            continue
                        runtime.node.enqueue_navigation(request_id, x, y, yaw)
                        await socket.send_json({
                            "type": "navigation", "accepted": True,
                            "id": request_id, "message": "Đã gửi mục tiêu tới ROS 2",
                        })
                        continue

                    if packet_type != "command":
                        continue
                    enabled = data.get("enabled") is True
                    snapshot = state.snapshot()
                    if enabled and snapshot["mission"] != "idle":
                        await socket.send_json({"type": "control", "accepted": False,
                                                "message": "Stop the active mission before manual control"})
                        continue
                    owner = control["owner"]
                    if enabled and owner not in (None, socket):
                        await socket.send_json({"type": "control", "accepted": False,
                                                "message": "Another dashboard client owns manual control"})
                        continue
                    if enabled:
                        control["owner"] = socket
                        state.set_command(data, True)
                    elif owner in (None, socket):
                        state.disable()
                        control["owner"] = None
                    await socket.send_json({"type": "control", "accepted": True, "enabled": enabled})
                except (KeyError, ValueError, TypeError, OverflowError,
                        json.JSONDecodeError) as error:
                    response_type = (
                        "navigation"
                        if isinstance(data, dict) and data.get("type") == "navigate"
                        else "control"
                    )
                    await socket.send_json({
                        "type": response_type, "accepted": False, "message": str(error),
                    })
        finally:
            clients.pop(socket, None)
            if control["owner"] is socket:
                state.disable()
                control["owner"] = None
        return socket

    async def telemetry_loop():
        """Fan out current telemetry at 20 Hz and map data only when changed."""
        while True:
            revision = state.snapshot()["map_revision"]
            stale = []
            for socket, delivered_revision in tuple(clients.items()):
                include_map = delivered_revision != revision
                packet = state.snapshot(include_map=include_map)
                try:
                    await socket.send_str(json.dumps(packet, allow_nan=False, separators=(",", ":")))
                    if include_map and socket in clients:
                        clients[socket] = revision
                except (ConnectionError, RuntimeError):
                    stale.append(socket)
            for socket in stale:
                clients.pop(socket, None)
            await asyncio.sleep(0.05)

    async def background(application):
        """Tie ROS startup/shutdown to the aiohttp application lifecycle."""
        runtime = RosRuntime(state)
        runtime_holder["runtime"] = runtime
        task = asyncio.create_task(telemetry_loop())
        yield
        state.disable()
        await asyncio.sleep(0.65)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        runtime.shutdown()
        runtime_holder["runtime"] = None

    app.router.add_get("/", index)
    app.router.add_static("/assets", ROOT / "assets")
    app.router.add_get("/health", health)
    app.router.add_get("/ws", websocket)
    app.cleanup_ctx.append(background)
    return app


if __name__ == "__main__":
    web.run_app(
        create_app(),
        host=os.getenv("HOST", "127.0.0.1"),
        port=int(os.getenv("PORT", "8080")),
        print=lambda line: print(line, flush=True),
    )
