# Brings up the bridge node (serial + ROS topics/services + built-in web UI).
# NO rosbridge needed — the node serves the UI and pushes telemetry itself.
#
#   ros2 launch rc_car_bridge bringup.launch.py serial_port:=/dev/ttyAMA0
#   then open  http://<pi-ip>:8080
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("cmd_rate_hz", default_value="20.0"),
        Node(
            package="rc_car_bridge", executable="uart_bridge",
            name="rc_car_uart_bridge", output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "baud": LaunchConfiguration("baud"),
                "http_port": LaunchConfiguration("http_port"),
                "cmd_rate_hz": LaunchConfiguration("cmd_rate_hz"),
            }],
        ),
    ])
