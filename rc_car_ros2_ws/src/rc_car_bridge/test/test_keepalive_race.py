"""Regression test for the keepalive-cancels-maneuver race.

Bug: ESP32 confirms "auto" (busy) only via telemetry, which arrives at 5 Hz
(every 200ms). The bridge's drive keepalive fires at 20 Hz (every 50ms). If
the bridge waits for telemetry to mark itself busy, there's up to a 200ms
window after sending turnAngle/moveDistance/gotoCoord/testMotor where the
keepalive still sees busy=False and sends drive(0,0) — which the ESP32's
drive() handler turns into clearAuto(), killing the maneuver almost instantly
(visible as "the wheel moves a tiny bit then stops").

Fix: mark busy=True locally the instant a maneuver command is sent, before
write() even returns — don't wait for telemetry to confirm it.

    cd ~/rc_car_ros2_ws/src/rc_car_bridge && python3 -m pytest test/test_keepalive_race.py -v
"""
import importlib.util
import os

_HERE = os.path.dirname(__file__)
_SPEC = importlib.util.spec_from_file_location(
    "standalone_bridge", os.path.join(_HERE, "..", "scripts", "standalone_bridge.py"))
sb = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(sb)


class _RecordingBridge(sb.Bridge):
    """Same Bridge class, minus real serial I/O — just records writes."""
    def __init__(self):
        self.invert_steer = False
        self.cmd = (0, 0)
        self.busy = False
        self.running = True
        self.sent = []

    def write(self, obj):
        self.sent.append(obj)
        return True


def _keepalive_tick(b):
    if b.busy:
        return
    t, s = b.cmd
    b.write({"cmd": "drive", "t": t, "s": s})


def test_busy_set_immediately_on_maneuver_send():
    b = _RecordingBridge()
    b.command({"cmd": "turnAngle", "degrees": 180, "speed": 160, "msPerDegree": 8.0})
    assert b.busy is True


def test_keepalive_suppressed_during_the_telemetry_confirmation_gap():
    """4 keepalive ticks (=200ms @ 20Hz) must NOT emit drive() before the
    ESP32's first confirming telemetry frame (5Hz) would even arrive."""
    b = _RecordingBridge()
    b.command({"cmd": "turnAngle", "degrees": 180, "speed": 160, "msPerDegree": 8.0})
    for _ in range(4):
        _keepalive_tick(b)
    drive_cmds = [c for c in b.sent if c.get("cmd") == "drive"]
    assert not drive_cmds, f"keepalive cancelled the maneuver: {drive_cmds}"


def test_busy_clears_when_telemetry_reports_auto_false():
    b = _RecordingBridge()
    b.command({"cmd": "turnAngle", "degrees": 90, "speed": 160, "msPerDegree": 8.0})
    assert b.busy is True
    # ESP32 confirms it's still going
    t = sb.decode_telemetry({"batt": 7.6, "auto": True, "pwmL": 160, "pwmR": 160})
    b.busy = t["busy"]
    assert b.busy is True
    # ESP32 reports the maneuver finished
    t = sb.decode_telemetry({"batt": 7.6, "auto": False, "pwmL": 0, "pwmR": 0})
    b.busy = t["busy"]
    assert b.busy is False
    _keepalive_tick(b)
    assert b.sent[-1] == {"cmd": "drive", "t": 0, "s": 0}


def test_stop_and_cancel_clear_busy_immediately():
    b = _RecordingBridge()
    b.command({"cmd": "turnAngle", "degrees": 90, "speed": 160, "msPerDegree": 8.0})
    assert b.busy is True
    b.command({"cmd": "stop"})
    assert b.busy is False
