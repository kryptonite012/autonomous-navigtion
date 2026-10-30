from glob import glob

from setuptools import setup

package_name = "nexus_hardware"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="ROS 2 base driver for NEXUS: odometry, range sensors, IMU, safety-gated cmd_vel",
    license="Apache-2.0",
    entry_points={"console_scripts": ["base_node = nexus_hardware.base_node:main"]},
)
