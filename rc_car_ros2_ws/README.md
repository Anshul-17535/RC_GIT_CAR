# rc_car_ros2_ws

Colcon workspace for the ESP32 RC car ROS 2 bridge.

```
src/
  rc_car_interfaces/   # Telemetry msg + TurnAngle/MoveDistance/GotoCoord/TestMotor srv
  rc_car_bridge/       # uart_bridge + web_server nodes, launch, web UI, tests
```
Build & run:
```bash
cd ~/rc_car_ros2_ws && colcon build && source install/setup.bash
ros2 launch rc_car_bridge bringup.launch.py serial_port:=/dev/ttyAMA0
```
See `src/rc_car_bridge/README.md` for full setup, wiring and tests.
