#!/usr/bin/env python3
# ============================================================================
# standalone_bridge.py — RC car bridge with NO ROS at all.
#
# Same serial protocol + same web UI as the ROS node, but runs as a plain
# Python program. Only dependency: pyserial.  Use this if installing ROS 2 on
# the Pi is painful; you can move to the ROS package later with no UI changes.
#
#   pip3 install pyserial --break-system-packages    # once
#   python3 standalone_bridge.py --port /dev/ttyAMA0 --http 8080
#   then open  http://<pi-ip>:8080
#
# Serves the sibling ../web directory (index.html, app.js, style.css).
# ============================================================================
import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial  # pyserial

WEB_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "web"))
_CTYPE = {".html": "text/html", ".js": "application/javascript",
          ".css": "text/css", ".ico": "image/x-icon"}


# ---------------------------------------------------------------- protocol
def _clamp(v, lo, hi):
    v = int(v)
    return lo if v < lo else hi if v > hi else v


def to_line(obj):
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")


def parse_line(line):
    if isinstance(line, (bytes, bytearray)):
        line = line.decode("utf-8", "replace")
    line = line.strip()
    if not line:
        return None
    try:
        obj = json.loads(line)
    except (ValueError, TypeError):
        return None
    return obj if isinstance(obj, dict) else None


def is_telemetry(o):
    return isinstance(o, dict) and "batt" in o and "cmd" not in o


def decode_telemetry(o):
    return {"battery": float(o.get("batt", 0.0)), "roll": float(o.get("roll", 0.0)),
            "pitch": float(o.get("pitch", 0.0)), "busy": bool(o.get("auto", False)),
            "pwm_left": int(o.get("pwmL", 0)), "pwm_right": int(o.get("pwmR", 0))}


# ------------------------------------------------------------------- bridge
class Bridge:
    def __init__(self, port, baud, cmd_rate_hz, invert_steer):
        self.port_name = port
        self.baud = baud
        self.invert_steer = invert_steer
        self.period = 1.0 / max(cmd_rate_hz, 1.0)
        self.ser = None
        self.lock = threading.Lock()
        self.cmd = (0, 0)
        self.busy = False
        self.running = True
        self.rx = b""
        self.latest = {"battery": 0.0, "roll": 0.0, "pitch": 0.0, "busy": False,
                       "pwm_left": 0, "pwm_right": 0}
        self.last_rx_ms = 0
        self.web_dir = WEB_DIR
        self._open()
        threading.Thread(target=self._read_loop, daemon=True).start()
        threading.Thread(target=self._keepalive_loop, daemon=True).start()
        # push sane defaults to the ESP32 (matches firmware power-on)
        for d in ({"cmd": "speed", "v": 200}, {"cmd": "trim", "v": 0},
                  {"cmd": "minpwm", "v": 70}, {"cmd": "slew", "v": 12},
                  {"cmd": "deadband", "v": 6}):
            self.write(d)

    def _open(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05)
            print(f"[bridge] serial open {self.port_name}@{self.baud}")
        except Exception as e:  # noqa: BLE001
            self.ser = None
            print(f"[bridge] serial open FAILED ({self.port_name}): {e}")

    def write(self, obj):
        line = to_line(obj)
        with self.lock:
            if self.ser is None:
                self._open()
            if self.ser is None:
                return False
            try:
                self.ser.write(line)
                return True
            except Exception as e:  # noqa: BLE001
                print("[bridge] write failed:", e)
                try:
                    self.ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self.ser = None
                return False

    def _read_loop(self):
        while self.running:
            ser = self.ser
            if ser is None:
                time.sleep(0.5)
                self._open()
                continue
            try:
                chunk = ser.read(256)
            except Exception:  # noqa: BLE001
                with self.lock:
                    try:
                        ser.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self.ser = None
                continue
            if not chunk:
                continue
            self.rx += chunk
            while b"\n" in self.rx:
                raw, self.rx = self.rx.split(b"\n", 1)
                o = parse_line(raw)
                if is_telemetry(o):
                    self.latest = decode_telemetry(o)
                    self.busy = self.latest["busy"]
                    self.last_rx_ms = int(time.time() * 1000)

    def _keepalive_loop(self):
        while self.running:
            if not self.busy:
                t, s = self.cmd
                self.write({"cmd": "drive", "t": t, "s": s})
            time.sleep(self.period)

    def status(self):
        age = (int(time.time() * 1000) - self.last_rx_ms) if self.last_rx_ms else -1
        s = dict(self.latest)
        s["serial"] = self.ser is not None
        s["age_ms"] = age
        return s

    def command(self, obj):
        if not isinstance(obj, dict):
            return False
        cmd = obj.get("cmd")
        if not isinstance(cmd, str):
            return False
        if cmd == "drive":
            t = _clamp(obj.get("t", 0), -255, 255)
            s = _clamp(obj.get("s", 0), -255, 255)
            if self.invert_steer:
                s = -s
            self.cmd = (t, s)
            return self.write({"cmd": "drive", "t": t, "s": s})
        if cmd in ("turnAngle", "moveDistance", "gotoCoord", "testMotor"):
            # Mark busy LOCALLY now — don't wait up to 200ms for the ESP32's
            # next telemetry frame to confirm "auto". Otherwise the keepalive
            # loop sends drive(0,0) in that gap and cancels the maneuver we
            # just started (ESP32 drive() calls clearAuto()).
            self.cmd = (0, 0)
            self.busy = True
        elif cmd in ("stop", "cancelAuto", "testStop"):
            self.cmd = (0, 0)
            self.busy = False
        return self.write(obj)


# --------------------------------------------------------------------- http
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    @property
    def b(self):
        return self.server.bridge

    def log_message(self, *_a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/events":
            return self._sse()
        name = "index.html" if path in ("/", "/index.html") else path.lstrip("/")
        if name not in ("index.html", "app.js", "style.css"):
            return self._send(404, "text/plain", b"not found")
        try:
            with open(os.path.join(self.b.web_dir, name), "rb") as f:
                body = f.read()
        except OSError:
            return self._send(404, "text/plain", b"missing web/ files")
        self._send(200, _CTYPE.get(os.path.splitext(name)[1], "text/plain"), body)

    def do_POST(self):
        if self.path.split("?", 1)[0] != "/cmd":
            return self._send(404, "text/plain", b"x")
        try:
            n = int(self.headers.get("Content-Length", "0"))
            obj = json.loads(self.rfile.read(n).decode()) if n else {}
        except (ValueError, TypeError):
            return self._send(400, "application/json", b'{"ok":false}')
        ok = self.b.command(obj)
        self._send(200, "application/json", json.dumps({"ok": bool(ok)}).encode())

    def _sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while self.b.running:
                self.wfile.write(f"data: {json.dumps(self.b.status())}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http", type=int, default=8080)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--invert-steer", action="store_true")
    a = ap.parse_args()

    bridge = Bridge(a.port, a.baud, a.rate, a.invert_steer)
    httpd = ThreadingHTTPServer(("0.0.0.0", a.http), Handler)
    httpd.daemon_threads = True
    httpd.bridge = bridge
    print(f"[bridge] UI on http://<pi-ip>:{a.http}  (Ctrl-C to quit)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.running = False
        bridge.write({"cmd": "stop"})
        httpd.shutdown()


if __name__ == "__main__":
    main()
