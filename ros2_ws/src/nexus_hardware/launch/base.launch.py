import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = os.environ.get("NEXUS_ROOT", os.path.expanduser("~/PROJECT-NEXUS"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(root, "config", "nexus.yaml")),
        SetEnvironmentVariable("NEXUS_ROOT", root),
        Node(package="nexus_hardware", executable="base_node", name="nexus_base", output="screen",
             parameters=[{"config": LaunchConfiguration("config"), "rate_hz": 20.0}]),
    ])
