# ============================================================================
# protocol.py — pure (no-ROS, no-serial) encode/decode for the ESP32 link.
#
# The ESP32 (RC_Car.ino handleCmd) speaks newline-delimited JSON both ways:
#   Pi  -> ESP32 : command lines, every object has a "cmd" key
#   ESP32 -> Pi  : telemetry lines {batt,roll,pitch,auto,pwmL,pwmR} (no "cmd")
#
# Keeping all wire-format knowledge here makes it trivial to unit-test and means
# the node never hand-rolls JSON. clamp ranges mirror motors.cpp exactly.
# ============================================================================
import json


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def clamp_pwm(v):
    return int(_clamp(int(v), -255, 255))


# ---- Command builders (return plain dicts) --------------------------------
def drive(throttle, steer):
    return {"cmd": "drive", "t": clamp_pwm(throttle), "s": clamp_pwm(steer)}


def stop():
    return {"cmd": "stop"}


def cancel_auto():
    return {"cmd": "cancelAuto"}


def save():
    return {"cmd": "save"}


def get_config():
    return {"cmd": "getConfig"}


def speed(v):
    return {"cmd": "speed", "v": int(_clamp(int(v), 0, 255))}


def trim(v):
    return {"cmd": "trim", "v": int(_clamp(int(v), -50, 50))}


def min_pwm(v):
    return {"cmd": "minpwm", "v": int(_clamp(int(v), 0, 200))}


def slew(v):
    return {"cmd": "slew", "v": int(_clamp(int(v), 1, 255))}


def deadband(v):
    return {"cmd": "deadband", "v": int(_clamp(int(v), 0, 40))}


def invert(motor_index, value):
    return {"cmd": "invert", "m": int(_clamp(int(motor_index), 0, 3)),
            "v": bool(value)}


def turn_angle(degrees, spd=160, ms_per_degree=8.0):
    return {"cmd": "turnAngle", "degrees": float(degrees),
            "speed": int(_clamp(int(spd), 0, 255)),
            "msPerDegree": float(ms_per_degree)}


def move_distance(meters, spd=160, ms_per_meter=1200.0):
    return {"cmd": "moveDistance", "meters": float(meters),
            "speed": int(_clamp(int(spd), 0, 255)),
            "msPerMeter": float(ms_per_meter)}


def goto_coord(x, y, spd=160, ms_per_meter=1200.0, ms_per_degree=8.0):
    return {"cmd": "gotoCoord", "x": float(x), "y": float(y),
            "speed": int(_clamp(int(spd), 0, 255)),
            "msPerMeter": float(ms_per_meter),
            "msPerDegree": float(ms_per_degree)}


def test_motor(motor_index, pwm, duration_ms=1500):
    return {"cmd": "testMotor", "m": int(_clamp(int(motor_index), 0, 3)),
            "pwm": clamp_pwm(pwm), "ms": int(_clamp(int(duration_ms), 0, 10000))}


def test_stop():
    return {"cmd": "testStop"}


# ---- Serialisation --------------------------------------------------------
def to_line(obj):
    """dict -> bytes ready for the UART (compact JSON + '\\n')."""
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")


def parse_line(line):
    """One received line (str or bytes) -> dict, or None if not valid JSON."""
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


def is_telemetry(obj):
    """ESP32 telemetry frames carry 'batt' and never a 'cmd' key."""
    return isinstance(obj, dict) and "batt" in obj and "cmd" not in obj


def decode_telemetry(obj):
    """Normalise a telemetry dict into typed fields with safe defaults."""
    return {
        "battery": float(obj.get("batt", 0.0)),
        "roll": float(obj.get("roll", 0.0)),
        "pitch": float(obj.get("pitch", 0.0)),
        "busy": bool(obj.get("auto", False)),
        "pwm_left": int(obj.get("pwmL", 0)),
        "pwm_right": int(obj.get("pwmR", 0)),
    }
