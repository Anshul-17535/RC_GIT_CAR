"""End-to-end bridge test over a pseudo-terminal — no ESP32 required.

Opens a PTY pair, points the node's serial_port at the slave, then:
  * writes a telemetry line into the PTY and checks the node publishes Telemetry
  * checks the node's 20 Hz keepalive writes a 'drive' command back

    python3 -m pytest test/test_uart_loopback.py -v
Skips automatically on platforms without pty (e.g. native Windows).
"""
import json
import os
import time

import pytest

pty = pytest.importorskip("pty")

rclpy = pytest.importorskip("rclpy")
from rc_car_interfaces.msg import Telemetry            # noqa: E402
from rc_car_bridge.uart_bridge_node import UartBridge  # noqa: E402


@pytest.fixture()
def ros_ctx():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def test_telemetry_publish_and_keepalive(ros_ctx):
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)

    node = UartBridge()
    # Re-point at our PTY and reopen (node opened the default port in __init__).
    node._port_name = slave_name
    node._open_serial()

    received = []
    node.create_subscription(
        Telemetry, "~/telemetry", lambda m: received.append(m), 10)

    # ESP32 -> Pi: one telemetry frame.
    frame = ('{"batt":7.4,"roll":3.0,"pitch":-1.0,"auto":false,'
             '"pwmL":40,"pwmR":-40}\n')
    os.write(master, frame.encode())

    _spin(node, 0.5)

    assert received, "node did not publish Telemetry from the PTY frame"
    assert abs(received[-1].battery - 7.4) < 1e-3
    assert received[-1].pwm_left == 40

    # Pi -> ESP32: the keepalive timer must emit a drive command.
    time.sleep(0.2)
    data = os.read(master, 4096).decode("utf-8", "replace")
    lines = [l for l in data.splitlines() if l.strip()]
    assert lines, "no keepalive command written by the node"
    last = json.loads(lines[-1])
    assert last["cmd"] == "drive"

    node.destroy_node()
    os.close(master)
