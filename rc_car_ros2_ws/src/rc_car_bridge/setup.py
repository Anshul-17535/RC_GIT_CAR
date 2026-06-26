import os
from glob import glob
from setuptools import find_packages, setup

package_name = "rc_car_bridge"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Bhivesh",
    maintainer_email="bhivesh@example.com",
    description="ROS 2 <-> ESP32 UART bridge with a built-in web control UI.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "uart_bridge = rc_car_bridge.uart_bridge_node:main",
        ],
    },
)
