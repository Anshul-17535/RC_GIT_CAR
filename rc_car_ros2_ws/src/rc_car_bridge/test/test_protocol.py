"""Unit tests for the wire protocol — no ROS, no hardware needed.

    cd ~/rc_car_ros2_ws/src/rc_car_bridge && python3 -m pytest test/test_protocol.py -v
"""
import json
from rc_car_bridge import protocol as P


def test_drive_clamps_and_keys():
    d = P.drive(999, -999)
    assert d == {"cmd": "drive", "t": 255, "s": -255}


def test_tuning_ranges():
    assert P.speed(999)["v"] == 255
    assert P.trim(-999)["v"] == -50
    assert P.min_pwm(999)["v"] == 200
    assert P.slew(0)["v"] == 1          # slew floor is 1
    assert P.deadband(999)["v"] == 40


def test_turn_move_goto_defaults():
    assert P.turn_angle(90)["msPerDegree"] == 8.0
    assert P.move_distance(0.5)["msPerMeter"] == 1200.0
    g = P.goto_coord(0.5, 0.5)
    assert g["cmd"] == "gotoCoord" and g["x"] == 0.5 and g["y"] == 0.5


def test_to_line_is_newline_terminated_json():
    line = P.to_line(P.stop())
    assert line.endswith(b"\n")
    assert json.loads(line) == {"cmd": "stop"}


def test_parse_and_classify_telemetry():
    raw = '{"batt":7.81,"roll":1.5,"pitch":-2.0,"auto":true,"pwmL":120,"pwmR":-118}'
    obj = P.parse_line(raw)
    assert P.is_telemetry(obj)
    t = P.decode_telemetry(obj)
    assert abs(t["battery"] - 7.81) < 1e-6
    assert t["busy"] is True
    assert t["pwm_left"] == 120 and t["pwm_right"] == -118


def test_command_echo_is_not_telemetry():
    # A command line (has "cmd") must never be mistaken for telemetry.
    assert not P.is_telemetry(P.parse_line('{"cmd":"drive","t":0,"s":0}'))


def test_parse_garbage_returns_none():
    assert P.parse_line("not json") is None
    assert P.parse_line("") is None
    assert P.parse_line("[1,2,3]") is None     # not a dict


def test_decode_telemetry_rich_fields():
    raw = ('{"batt":7.6,"roll":1.0,"pitch":2.0,"yaw":33.0,"gx":0.1,"gy":0.2,'
           '"gz":0.3,"ax":0.0,"ay":0.0,"az":1.0,"imu":true,"auto":false,'
           '"pwmL":10,"pwmR":-10,"hh":true,"tgt":90.0,"err":-5.0,"out":40.0}')
    obj = P.parse_line(raw)
    assert P.is_telemetry(obj)
    t = P.decode_telemetry(obj)
    assert t["yaw"] == 33.0 and t["imu"] is True
    assert t["hh"] is True and t["tgt"] == 90.0
    assert t["gz"] == 0.3 and t["az"] == 1.0


def test_is_config_detects_config_frames():
    cfg = P.parse_line('{"type":"config","maxSpeed":200,"mt":[100,100,100,100],"kp":4.0}')
    assert P.is_config(cfg)
    assert not P.is_config(P.parse_line('{"batt":7.6}'))
    assert not P.is_config(P.parse_line('{"cmd":"drive","t":0,"s":0}'))
