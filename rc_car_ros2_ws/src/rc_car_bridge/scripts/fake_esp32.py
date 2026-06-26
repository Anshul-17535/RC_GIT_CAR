#!/usr/bin/env python3
"""Pretend to be the ESP32 — NO hardware needed. Streams telemetry at 5 Hz and
prints every command it receives. Lets you test the whole ROS stack + web UI on
a laptop/Pi with a virtual serial pair:

    sudo apt install socat
    socat -d -d pty,raw,echo=0 pty,raw,echo=0
      # prints two device names, e.g. /dev/pts/3  and  /dev/pts/4

    python3 fake_esp32.py /dev/pts/4              # one end = fake car
    ros2 launch rc_car_bridge bringup.launch.py serial_port:=/dev/pts/3

The fake mirrors any drive t/s into pwmL/pwmR and fakes a slow roll/pitch sweep
so the UI's IMU horizon visibly moves.
"""
import json
import math
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/pts/4"
ser = serial.Serial(port, 115200, timeout=0.02)
print(f"fake ESP32 on {port}")

last_tx = 0.0
pwmL = pwmR = 0
busy_until = 0.0
buf = b""
t0 = time.time()

while True:
    # read commands
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
            pwmL, pwmR = max(-255, min(255, t + s)), max(-255, min(255, t - s))
        elif cmd in ("turnAngle", "moveDistance", "gotoCoord"):
            busy_until = time.time() + 1.5
            print("  [fake] maneuver:", cmd, {k: v for k, v in c.items() if k != "cmd"})
        elif cmd == "stop":
            pwmL = pwmR = 0
        else:
            print("  [fake] cmd:", json.dumps(c))

    # 5 Hz telemetry
    now = time.time()
    if now - last_tx >= 0.2:
        last_tx = now
        roll = 20.0 * math.sin((now - t0) * 0.7)
        pitch = 10.0 * math.cos((now - t0) * 0.5)
        frame = {
            "batt": round(7.6 + 0.05 * math.sin(now), 2),
            "roll": round(roll, 1), "pitch": round(pitch, 1),
            "auto": now < busy_until,
            "pwmL": pwmL, "pwmR": pwmR,
        }
        ser.write((json.dumps(frame, separators=(",", ":")) + "\n").encode())
