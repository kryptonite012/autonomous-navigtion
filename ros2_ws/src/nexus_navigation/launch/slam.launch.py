"""Online SLAM with SLAM Toolbox. Requires a LiDAR publishing /scan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("nexus_navigation"), "config", "slam_toolbox.yaml")
    return LaunchDescription([
        Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
             parameters=[params], output="screen"),
    ])
