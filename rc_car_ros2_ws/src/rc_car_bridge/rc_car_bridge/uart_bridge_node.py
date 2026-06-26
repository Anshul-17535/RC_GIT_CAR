#!/usr/bin/env python3
# ============================================================================
# uart_bridge_node.py — ROS 2 <-> ESP32 UART bridge for the RC car, with a
# BUILT-IN web UI. No rosbridge, no extra packages: the node serves the static
# UI over HTTP, pushes telemetry via Server-Sent Events (SSE), and takes
# commands via POST /cmd. Stdlib http.server + pyserial only.
#
#   ROS subs   : ~/cmd_vel            geometry_msgs/Twist (linear.x, angular.z)
#   ROS pubs   : ~/telemetry  Telemetry | ~/battery Float32 | ~/busy Bool
#                ~/imu  sensor_msgs/Imu  (orientation from roll/pitch)
#   ROS srvs   : ~/turn_angle ~/move_distance ~/goto_coord ~/test_motor
#                ~/stop ~/cancel_auto ~/save_config
#   HTTP       : GET /            -> web/index.html
#                GET /style.css /app.js
#                GET /events      -> text/event-stream telemetry (SSE)
#                POST /cmd        -> body is an ESP32 JSON command, forwarded
#   Params     : serial_port baud cmd_rate_hz throttle_scale steer_scale
#                invert_steer publish_imu frame_id http_port web_dir
#                max_speed trim min_pwm slew deadband (pushed to ESP32)
#
# Safety (matched to RC_Car.ino / motors.cpp):
#   * ESP32 stops motors if silent > 600 ms -> we stream last drive at
#     cmd_rate_hz (20 Hz). Bridge dies -> car stops within 600 ms (intended).
#   * ESP32 drive() cancels timed maneuvers -> we SUPPRESS the drive keepalive
#     while telemetry reports busy (ESP32 also disables its failsafe then).
# ============================================================================
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from ament_index_python.packages import get_package_share_directory

import serial  # pyserial

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from rc_car_interfaces.msg import Telemetry
from rc_car_interfaces.srv import TurnAngle, MoveDistance, GotoCoord, TestMotor

from . import protocol as P

_CTYPE = {".html": "text/html", ".js": "application/javascript",
          ".css": "text/css", ".ico": "image/x-icon"}


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


# ----------------------------------------------------------------- HTTP layer
class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    @property
    def _node(self):
        return self.server.node          # set on the server instance

    def log_message(self, *_args):       # silence default stderr spam
        pass

    def _send_bytes(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/events":
            return self._stream_events()
        if path in ("/", "/index.html"):
            return self._send_file("index.html")
        if path in ("/style.css", "/app.js"):
            return self._send_file(path.lstrip("/"))
        self._send_bytes(404, "text/plain", b"not found")

    def do_POST(self):
        if self.path.split("?", 1)[0] != "/cmd":
            return self._send_bytes(404, "text/plain", b"not found")
        try:
            n = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(n) if n else b""
            obj = json.loads(body.decode("utf-8")) if body else {}
        except (ValueError, TypeError):
            return self._send_bytes(400, "application/json", b'{"ok":false}')
        ok = self._node.handle_ui_command(obj)
        self._send_bytes(200, "application/json",
                         json.dumps({"ok": bool(ok)}).encode())

    def _send_file(self, name):
        fp = os.path.join(self._node.web_dir, name)
        try:
            with open(fp, "rb") as f:
                body = f.read()
        except OSError:
            return self._send_bytes(404, "text/plain", b"missing")
        ext = os.path.splitext(name)[1]
        self._send_bytes(200, _CTYPE.get(ext, "application/octet-stream"), body)

    def _stream_events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while self._node.running:
                payload = json.dumps(self._node.latest_status())
                self.wfile.write(f"data: {payload}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.1)                  # 10 Hz push
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


# --------------------------------------------------------------------- node
class UartBridge(Node):
    def __init__(self):
        super().__init__("rc_car_uart_bridge")

        self.declare_parameter("serial_port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("throttle_scale", 255.0)
        self.declare_parameter("steer_scale", 255.0)
        self.declare_parameter("invert_steer", False)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("web_dir", "")
        self.declare_parameter("max_speed", 200)
        self.declare_parameter("trim", 0)
        self.declare_parameter("min_pwm", 70)
        self.declare_parameter("slew", 12)
        self.declare_parameter("deadband", 6)

        gp = self.get_parameter
        self._port_name = gp("serial_port").value
        self._baud = int(gp("baud").value)
        self._rate = float(gp("cmd_rate_hz").value)
        self._tscale = float(gp("throttle_scale").value)
        self._sscale = float(gp("steer_scale").value)
        self._invert_steer = bool(gp("invert_steer").value)
        self._publish_imu = bool(gp("publish_imu").value)
        self._frame_id = gp("frame_id").value
        self._http_port = int(gp("http_port").value)
        self.web_dir = gp("web_dir").value or os.path.join(
            get_package_share_directory("rc_car_bridge"), "web")

        # state
        self._ser = None
        self._ser_lock = threading.Lock()
        self._cmd = (0, 0)
        self._busy = False
        self.running = True
        self._rx_buf = b""
        self._latest = {"battery": 0.0, "roll": 0.0, "pitch": 0.0,
                        "busy": False, "pwm_left": 0, "pwm_right": 0}
        self._last_rx_ms = 0

        # ROS pubs
        self.pub_tele = self.create_publisher(Telemetry, "~/telemetry", 10)
        self.pub_batt = self.create_publisher(Float32, "~/battery", 10)
        self.pub_busy = self.create_publisher(Bool, "~/busy", 10)
        self.pub_imu = self.create_publisher(Imu, "~/imu", 10)

        # ROS subs / services
        self.create_subscription(Twist, "~/cmd_vel", self._on_cmd_vel, 10)
        self.create_service(TurnAngle, "~/turn_angle", self._srv_turn)
        self.create_service(MoveDistance, "~/move_distance", self._srv_move)
        self.create_service(GotoCoord, "~/goto_coord", self._srv_goto)
        self.create_service(TestMotor, "~/test_motor", self._srv_test)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_service(Trigger, "~/cancel_auto", self._srv_cancel)
        self.create_service(Trigger, "~/save_config", self._srv_save)
        self.add_on_set_parameters_callback(self._on_set_params)

        # serial + reader thread
        self._open_serial()
        threading.Thread(target=self._read_loop, daemon=True).start()

        # keepalive streamer
        self.create_timer(1.0 / max(self._rate, 1.0), self._tick)

        # embedded HTTP server
        self._httpd = ThreadingHTTPServer(("0.0.0.0", self._http_port), _Handler)
        self._httpd.daemon_threads = True
        self._httpd.node = self
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

        self._push_tuning()
        self.get_logger().info(
            f"bridge up: serial {self._port_name}@{self._baud}, "
            f"UI http://<pi-ip>:{self._http_port}, drive {self._rate:.0f} Hz")

    # ---- serial ----
    def _open_serial(self):
        try:
            self._ser = serial.Serial(self._port_name, self._baud, timeout=0.05)
            self.get_logger().info(f"serial open: {self._port_name}")
        except Exception as e:  # noqa: BLE001
            self._ser = None
            self.get_logger().error(f"serial open FAILED ({self._port_name}): {e}")

    def _write(self, obj):
        line = P.to_line(obj)
        with self._ser_lock:
            if self._ser is None:
                self._open_serial()
            if self._ser is None:
                return False
            try:
                self._ser.write(line)
                return True
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"serial write failed: {e}")
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = None
                return False

    def _read_loop(self):
        while self.running:
            ser = self._ser
            if ser is None:
                time.sleep(0.5)
                self._open_serial()
                continue
            try:
                chunk = ser.read(256)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"serial read failed: {e}")
                with self._ser_lock:
                    try:
                        ser.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self._ser = None
                continue
            if not chunk:
                continue
            self._rx_buf += chunk
            while b"\n" in self._rx_buf:
                raw, self._rx_buf = self._rx_buf.split(b"\n", 1)
                obj = P.parse_line(raw)
                if P.is_telemetry(obj):
                    self._publish_telemetry(P.decode_telemetry(obj))

    # ---- publishing ----
    def _publish_telemetry(self, t):
        self._busy = t["busy"]
        self._latest = t
        self._last_rx_ms = int(time.time() * 1000)
        now = self.get_clock().now().to_msg()

        m = Telemetry()
        m.header.stamp = now
        m.header.frame_id = self._frame_id
        m.battery = t["battery"]; m.roll = t["roll"]; m.pitch = t["pitch"]
        m.busy = t["busy"]
        m.pwm_left = int(t["pwm_left"]); m.pwm_right = int(t["pwm_right"])
        self.pub_tele.publish(m)
        self.pub_batt.publish(Float32(data=t["battery"]))
        self.pub_busy.publish(Bool(data=t["busy"]))

        if self._publish_imu:
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = self._frame_id
            qx, qy, qz, qw = _rpy_to_quat(
                math.radians(t["roll"]), math.radians(t["pitch"]), 0.0)
            imu.orientation.x = qx; imu.orientation.y = qy
            imu.orientation.z = qz; imu.orientation.w = qw
            imu.orientation_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, -1.0]
            imu.angular_velocity_covariance = [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
            imu.linear_acceleration_covariance = [-1.0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.pub_imu.publish(imu)

    def latest_status(self):
        age = (int(time.time() * 1000) - self._last_rx_ms) if self._last_rx_ms else -1
        s = dict(self._latest)
        s["serial"] = self._ser is not None
        s["age_ms"] = age
        return s

    # ---- inputs ----
    def _on_cmd_vel(self, msg: Twist):
        t = P.clamp_pwm(msg.linear.x * self._tscale)
        s = P.clamp_pwm(msg.angular.z * self._sscale)
        if self._invert_steer:
            s = -s
        self._cmd = (t, s)

    def _tick(self):
        if self._busy:
            return
        t, s = self._cmd
        self._write(P.drive(t, s))

    def handle_ui_command(self, obj):
        """Forward a command object from the web UI to the ESP32."""
        if not isinstance(obj, dict):
            return False
        cmd = obj.get("cmd")
        if not isinstance(cmd, str):
            return False
        if cmd == "drive":
            t = P.clamp_pwm(obj.get("t", 0))
            s = P.clamp_pwm(obj.get("s", 0))
            if self._invert_steer:
                s = -s
            self._cmd = (t, s)
            return self._write(P.drive(t, s))
        if cmd in ("turnAngle", "moveDistance", "gotoCoord",
                   "testMotor", "stop", "cancelAuto"):
            self._cmd = (0, 0)
        return self._write(obj)

    # ---- ROS services (for the PID/nav stack) ----
    def _srv_turn(self, req, resp):
        self._cmd = (0, 0)
        ok = self._write(P.turn_angle(req.degrees, req.speed or 160,
                                      req.ms_per_degree or 8.0))
        resp.accepted = ok; resp.message = "sent" if ok else "serial down"
        return resp

    def _srv_move(self, req, resp):
        self._cmd = (0, 0)
        ok = self._write(P.move_distance(req.meters, req.speed or 160,
                                         req.ms_per_meter or 1200.0))
        resp.accepted = ok; resp.message = "sent" if ok else "serial down"
        return resp

    def _srv_goto(self, req, resp):
        self._cmd = (0, 0)
        ok = self._write(P.goto_coord(req.x, req.y, req.speed or 160,
                                      req.ms_per_meter or 1200.0,
                                      req.ms_per_degree or 8.0))
        resp.accepted = ok; resp.message = "sent" if ok else "serial down"
        return resp

    def _srv_test(self, req, resp):
        self._cmd = (0, 0)
        ok = self._write(P.test_motor(req.motor, req.pwm, req.duration_ms or 1500))
        resp.accepted = ok; resp.message = "sent" if ok else "serial down"
        return resp

    def _srv_stop(self, req, resp):
        self._cmd = (0, 0); ok = self._write(P.stop())
        resp.success = ok; resp.message = "stopped" if ok else "serial down"
        return resp

    def _srv_cancel(self, req, resp):
        self._cmd = (0, 0); ok = self._write(P.cancel_auto())
        resp.success = ok; resp.message = "cancelled" if ok else "serial down"
        return resp

    def _srv_save(self, req, resp):
        ok = self._write(P.save())
        resp.success = ok; resp.message = "saved" if ok else "serial down"
        return resp

    # ---- params ----
    def _push_tuning(self):
        gp = self.get_parameter
        self._write(P.speed(gp("max_speed").value))
        self._write(P.trim(gp("trim").value))
        self._write(P.min_pwm(gp("min_pwm").value))
        self._write(P.slew(gp("slew").value))
        self._write(P.deadband(gp("deadband").value))

    def _on_set_params(self, params):
        mapping = {"max_speed": P.speed, "trim": P.trim, "min_pwm": P.min_pwm,
                   "slew": P.slew, "deadband": P.deadband}
        for prm in params:
            if prm.name in mapping and prm.type_ in (
                    Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                self._write(mapping[prm.name](prm.value))
            elif prm.name == "invert_steer" and prm.type_ == Parameter.Type.BOOL:
                self._invert_steer = bool(prm.value)
        return SetParametersResult(successful=True)

    # ---- cleanup ----
    def destroy_node(self):
        self.running = False
        try:
            self._write(P.stop())
        except Exception:  # noqa: BLE001
            pass
        try:
            self._httpd.shutdown()
        except Exception:  # noqa: BLE001
            pass
        with self._ser_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UartBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
