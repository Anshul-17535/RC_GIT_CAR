# rc_car_bridge

## ⚡ Option A — run it NOW with NO ROS (recommended if ROS install is fighting you)
The standalone bridge is the same serial protocol + the same web UI, as a plain
Python program. Only needs pyserial.

```bash
pip3 install pyserial --break-system-packages
cd ~/Desktop/robot/RC_GIT/RC_GIT_CAR/rc_car_ros2_ws/src/rc_car_bridge
python3 scripts/standalone_bridge.py --port /dev/ttyAMA0 --http 8080
# open http://<pi-ip>:8080
```
No colcon, no rosbridge, no ROS. The ROS package (Option B) stays available for
when you want `~/cmd_vel`, `~/imu`, services, etc. — the UI is identical.

Test it with no car first (two terminals):
```bash
sudo apt install -y socat
socat -d -d pty,raw,echo=0 pty,raw,echo=0          # prints two /dev/pts/N
python3 scripts/fake_esp32.py /dev/pts/4
python3 scripts/standalone_bridge.py --port /dev/pts/3 --http 8080
```

---

## "Unable to locate package python3-colcon-common-extensions"
That package lives in the **ROS 2 apt repo**. If apt can't find it, the repo
isn't configured (or this Pi isn't Ubuntu 24.04). Fixes:

* If `ros2` already works here: `pip3 install -U colcon-common-extensions --break-system-packages`
* Add the ROS 2 apt repo (Ubuntu 24.04 / Jazzy, current 2025 method):
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
export RUV=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep tag_name | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${RUV}/ros2-apt-source_${RUV}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```
* If this Pi runs **Raspberry Pi OS / Debian**, there are no native Jazzy debs.
  Use Option A above, or run the ROS package inside a `ros:jazzy` Docker image.

---

# rc_car_bridge — ROS 2 (Jazzy) ↔ ESP32 RC car  (no rosbridge)

Bridges your ESP32 firmware (`RC_Car.ino` + `pilink.*`) to ROS 2 over UART and
**serves its own web UI** — drive the car, run the timed maneuvers, tune motors
live, and watch the IMU + telemetry. The UI talks to the node directly over
plain HTTP, so there is **no rosbridge and nothing extra to pip/apt install**
beyond `pyserial`.

```
 web UI ──HTTP (same node)──▶ uart_bridge_node ──UART 115200──▶ ESP32
   ▲  POST /cmd  (commands)        │  ROS topics/services        JSON lines
   └──  GET /events (SSE telem) ───┘  for your PID/nav stack
```

## Wiring (3.3 V both sides — no level shifter)
| Pi (40-pin)         | ESP32          |
|---------------------|----------------|
| GPIO14 TXD (pin 8)  | RX2  GPIO16    |
| GPIO15 RXD (pin 10) | TX2  GPIO17    |
| GND (pin 6)         | GND (required) |

## One-time Pi UART setup (Ubuntu 24.04 / Jazzy)
```bash
sudo usermod -aG dialout $USER          # then log out/in
sudo nano /boot/firmware/config.txt     # add:  enable_uart=1   dtoverlay=disable-bt
sudo systemctl disable --now serial-getty@ttyAMA0.service
ls -l /dev/serial*                      # device is usually /dev/ttyAMA0
```
> Raspberry Pi OS: file is `/boot/config.txt`, device may be `/dev/serial0`.

## Install (note: NO rosbridge)
```bash
sudo apt update
sudo apt install -y python3-serial python3-colcon-common-extensions

mkdir -p ~/rc_car_ros2_ws/src
# copy rc_car_interfaces/ and rc_car_bridge/ into ~/rc_car_ros2_ws/src/
cd ~/rc_car_ros2_ws
colcon build
source install/setup.bash
```

## Run
```bash
ros2 launch rc_car_bridge bringup.launch.py serial_port:=/dev/ttyAMA0
```
Open **http://<pi-ip>:8080** on any device on the same network. That's it.

(Or run the node directly, no launch file:)
```bash
ros2 run rc_car_bridge uart_bridge --ros-args -p serial_port:=/dev/ttyAMA0 -p http_port:=8080
```

## Web UI ↔ node HTTP API (only two endpoints)
| Method | Path       | Purpose                                                        |
|--------|------------|----------------------------------------------------------------|
| GET    | `/events`  | Server-Sent Events stream of telemetry JSON (10 Hz)            |
| POST   | `/cmd`     | body = one ESP32 command, e.g. `{"cmd":"drive","t":120,"s":0}` |

The SSE payload is `{battery,roll,pitch,busy,pwm_left,pwm_right,serial,age_ms}`.
`POST /cmd` accepts the same vocabulary as the firmware: `drive, stop,
cancelAuto, turnAngle, moveDistance, gotoCoord, speed, trim, minpwm, slew,
deadband, invert, testMotor, testStop, save`.

## ROS interface (still there for your PID/nav code)
Namespace `/rc_car_uart_bridge`:
| Kind | Name | Type |
|------|------|------|
| sub  | `~/cmd_vel` | geometry_msgs/Twist (linear.x, angular.z normalised −1..1) |
| pub  | `~/telemetry` `~/battery` `~/busy` `~/imu` | Telemetry / Float32 / Bool / sensor_msgs/Imu |
| srv  | `~/turn_angle` `~/move_distance` `~/goto_coord` `~/test_motor` | rc_car_interfaces/* |
| srv  | `~/stop` `~/cancel_auto` `~/save_config` | std_srvs/Trigger |

```bash
ros2 topic pub -r 10 /rc_car_uart_bridge/cmd_vel geometry_msgs/Twist "{linear: {x: 0.6}}"
ros2 service call /rc_car_uart_bridge/turn_angle rc_car_interfaces/srv/TurnAngle "{degrees: 90}"
ros2 topic echo /rc_car_uart_bridge/imu
```

## Tests
```bash
cd ~/rc_car_ros2_ws/src/rc_car_bridge
python3 -m pytest test/test_protocol.py -v        # pure protocol, no hardware
# loopback needs the built workspace sourced (uses rc_car_interfaces):
python3 -m pytest test/test_uart_loopback.py -v   # PTY serial loopback, no ESP32
```

## Test the full stack with NO car (virtual serial)
```bash
sudo apt install -y socat
socat -d -d pty,raw,echo=0 pty,raw,echo=0          # prints two /dev/pts/N names
python3 src/rc_car_bridge/scripts/fake_esp32.py /dev/pts/4
ros2 launch rc_car_bridge bringup.launch.py serial_port:=/dev/pts/3
# open http://localhost:8080 — joystick moves pwmL/pwmR, IMU horizon sweeps
```

## First-light hardware check (no ROS)
```bash
python3 src/rc_car_bridge/scripts/serial_check.py /dev/ttyAMA0 115200
```

## Baked-in safety / gotchas
* **Keepalive:** ESP32 stops motors after 600 ms of silence. The node streams
  the last drive at `cmd_rate_hz` (20 Hz). Node dies → car stops in 600 ms.
* **Maneuvers vs drive:** ESP32 `drive()` cancels timed maneuvers, so the node
  suppresses the drive keepalive while telemetry reports `busy:true`.
* **IMU values:** `roll`/`pitch` are forwarded from ESP32 telemetry; they read 0
  until you wire the MPU and fill `Telemetry::roll()/pitch()` in firmware. No
  Pi change needed when you do.
* Tuning from the UI writes straight to the ESP32; the ROS `max_speed/trim/…`
  params are pushed once on boot.
