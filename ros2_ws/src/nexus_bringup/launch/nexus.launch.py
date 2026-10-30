"""Bring up the whole robot on ROS 2.

  ros2 launch nexus_bringup nexus.launch.py mode:=mapping        # SLAM Toolbox (needs LiDAR)
  ros2 launch nexus_bringup nexus.launch.py mode:=navigation map:=/path/map.yaml
  ros2 launch nexus_bringup nexus.launch.py mode:=sorting

The camera can only be opened by one process, so the vision node runs in
mapping/navigation modes and the sorting node (which has its own vision
pipeline) in sorting mode.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = os.environ.get("NEXUS_ROOT", os.path.expanduser("~/PROJECT-NEXUS"))
    cfg = LaunchConfiguration("config")
    share = get_package_share_directory

    def include(pkg, name, args=None):
        return PythonLaunchDescriptionSource(os.path.join(share(pkg), "launch", name)), args or {}

    desc_src, _ = include("nexus_description", "description.launch.py")
    slam_src, _ = include("nexus_navigation", "slam.launch.py")
    nav_src, _ = include("nexus_navigation", "navigation.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mapping", choices=["mapping", "navigation", "sorting", "base"]),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("config", default_value=os.path.join(root, "config", "nexus.yaml")),
        DeclareLaunchArgument("lidar", default_value="true",
                              description="start rplidar_ros (recommended over the built-in driver)"),
        SetEnvironmentVariable("NEXUS_ROOT", root),

        IncludeLaunchDescription(desc_src),
        Node(package="nexus_hardware", executable="base_node", name="nexus_base", output="screen",
             parameters=[{"config": cfg, "publish_scan": False}]),
        Node(package="rplidar_ros", executable="rplidar_node", name="rplidar", output="screen",
             condition=IfCondition(LaunchConfiguration("lidar")),
             parameters=[{"serial_port": "/dev/ttyUSB0", "frame_id": "laser", "angle_compensate": True}]),

        IncludeLaunchDescription(slam_src, condition=LaunchConfigurationEquals("mode", "mapping")),
        IncludeLaunchDescription(nav_src, condition=LaunchConfigurationEquals("mode", "navigation"),
                                 launch_arguments={"map": LaunchConfiguration("map")}.items()),

        Node(package="nexus_vision", executable="vision_node", output="screen",
             parameters=[{"config": cfg}], condition=LaunchConfigurationNotEquals("mode", "sorting")),
        Node(package="nexus_sorting", executable="sorting_node", output="screen",
             parameters=[{"config": cfg}], condition=LaunchConfigurationEquals("mode", "sorting")),
    ])
