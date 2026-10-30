"""Nav2 with a saved map (AMCL localisation). Pass map:=/path/to/map.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params = os.path.join(get_package_share_directory("nexus_navigation"), "config", "nav2_params.yaml")
    nav2 = get_package_share_directory("nav2_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("map", description="map yaml saved from the dashboard or slam_toolbox"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "bringup_launch.py")),
            launch_arguments={"map": LaunchConfiguration("map"), "params_file": params,
                              "use_sim_time": LaunchConfiguration("use_sim_time"),
                              "autostart": "true"}.items()),
    ])
