#!/usr/bin/env python3
"""Quick ESP32 link check — NO ROS. Sends a tiny wiggle, prints telemetry.

    python3 serial_check.py /dev/ttyAMA0 115200

Ctrl-C to stop (sends a final stop). Use this for first-light wiring tests:
confirms TX/RX are crossed and the common ground is in place.
"""
import json
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

ser = serial.Serial(port, baud, timeout=0.1)
print(f"open {port}@{baud}")


def send(d):
    ser.write((json.dumps(d, separators=(",", ":")) + "\n").encode())


buf = b""
t0 = time.time()
try:
    while True:
        # gentle forward nudge for 1.5 s, then stop, then repeat — keepalive 20 Hz
        phase = (time.time() - t0) % 3.0
        send({"cmd": "drive", "t": 90 if phase < 1.5 else 0, "s": 0})
        time.sleep(0.05)
        buf += ser.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if "batt" in obj:
                print(f"batt={obj.get('batt'):.2f}V roll={obj.get('roll')} "
                      f"pitch={obj.get('pitch')} auto={obj.get('auto')} "
                      f"pwmL={obj.get('pwmL')} pwmR={obj.get('pwmR')}")
except KeyboardInterrupt:
    send({"cmd": "stop"})
    print("\nstopped.")
