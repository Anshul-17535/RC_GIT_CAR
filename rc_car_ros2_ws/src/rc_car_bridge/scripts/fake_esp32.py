#!/usr/bin/env python3
"""Pretend to be the ESP32 RC car — NO hardware. Simulates the FULL protocol:
richer IMU telemetry (roll/pitch/yaw + gyro/accel), config snapshots on
getConfig/save, per-motor trim/invert, and a heading-hold PID that actually
drives yaw toward the target. Lets you exercise the entire UI on a laptop/Pi
with a virtual serial pair:

    sudo apt install socat
    socat -d -d pty,raw,echo=0 pty,raw,echo=0     # prints two /dev/pts/N names
    python3 fake_esp32.py /dev/pts/4
    ros2 launch rc_car_bridge bringup.launch.py serial_port:=/dev/pts/3
"""
import json
import math
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/pts/4"
ser = serial.Serial(port, 115200, timeout=0.02)
print(f"fake ESP32 (full sim) on {port}")

# persisted-ish config
cfg = {"maxSpeed": 200, "trim": 0, "minPwm": 70, "slew": 12, "deadband": 6,
       "imu": True, "inv": [False, False, False, False],
       "mt": [100, 100, 100, 100], "kp": 4.0, "ki": 0.0, "kd": 0.6}

pwmL = pwmR = 0
busy_until = 0.0
hh = False
tgt = 0.0
yaw = 0.0
buf = b""
t0 = time.time()
last_tx = 0.0


def send(obj):
    ser.write((json.dumps(obj, separators=(",", ":")) + "\n").encode())


def send_config():
    c = dict(cfg); c["type"] = "config"; send(c)


while True:
    buf += ser.read(256)
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        try:
            c = json.loads(line)
        except ValueError:
            continue
        cmd = c.get("cmd", "")
        if cmd == "drive":
            t, s = c.get("t", 0), c.get("s", 0)
            pwmL = max(-255, min(255, t + s)); pwmR = max(-255, min(255, t - s))
        elif cmd in ("turnAngle", "moveDistance", "gotoCoord"):
            busy_until = time.time() + 1.5
            print("  [fake] maneuver", cmd, {k: v for k, v in c.items() if k != "cmd"})
        elif cmd == "testMotor":
            busy_until = time.time() + (c.get("ms", 1500) / 1000.0)
            print("  [fake] testMotor", c.get("m"), c.get("pwm"))
        elif cmd == "stop":
            pwmL = pwmR = 0; hh = False
        elif cmd == "headingHold":
            hh = bool(c.get("enable", False))
            if hh:
                tgt = float(c.get("target", yaw)); print(f"  [fake] heading-hold ON tgt={tgt}")
            else:
                pwmL = pwmR = 0; print("  [fake] heading-hold OFF")
        elif cmd == "zeroYaw":
            yaw = 0.0; tgt = 0.0 if hh else tgt
        elif cmd == "speed":    cfg["maxSpeed"] = c.get("v", cfg["maxSpeed"])
        elif cmd == "trim":     cfg["trim"] = c.get("v", cfg["trim"])
        elif cmd == "minpwm":   cfg["minPwm"] = c.get("v", cfg["minPwm"])
        elif cmd == "slew":     cfg["slew"] = c.get("v", cfg["slew"])
        elif cmd == "deadband": cfg["deadband"] = c.get("v", cfg["deadband"])
        elif cmd == "invert":   cfg["inv"][c.get("m", 0)] = bool(c.get("v", False))
        elif cmd == "motorTrim": cfg["mt"][c.get("m", 0)] = c.get("v", 100)
        elif cmd == "pid":
            cfg["kp"] = c.get("kp", cfg["kp"]); cfg["ki"] = c.get("ki", cfg["ki"])
            cfg["kd"] = c.get("kd", cfg["kd"])
        elif cmd == "getConfig":
            send_config()
        elif cmd == "save":
            print("  [fake] SAVE"); send_config()

    now = time.time()
    err = 0.0
    if hh:
        # simulate the PID pulling yaw toward the target
        err = ((tgt - yaw + 180) % 360) - 180
        yaw += err * 0.08
    else:
        # idle drift so the cube visibly moves in the demo
        yaw = 25.0 * math.sin((now - t0) * 0.3)

    if now - last_tx >= 0.05:        # 20 Hz telemetry
        last_tx = now
        frame = {
            "batt": round(7.6 + 0.05 * math.sin(now), 2),
            "roll": round(18.0 * math.sin((now - t0) * 0.7), 1),
            "pitch": round(10.0 * math.cos((now - t0) * 0.5), 1),
            "yaw": round(yaw, 1),
            "gx": round(2.0 * math.cos(now), 2), "gy": round(1.5 * math.sin(now), 2),
            "gz": round(err * 0.08 / 0.05 if hh else 0.0, 2),
            "ax": 0.0, "ay": 0.0, "az": 1.0, "imu": cfg["imu"],
            "auto": now < busy_until,
            "pwmL": pwmL, "pwmR": pwmR,
            "hh": hh, "tgt": round(tgt, 1), "err": round(err, 1),
            "out": round(cfg["kp"] * err, 1),
        }
        send(frame)
