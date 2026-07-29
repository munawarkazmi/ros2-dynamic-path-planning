#!/usr/bin/env python3
"""Nav2 planner server with this package's global planners.

    ros2 launch ros2_dynamic_path_planning planner_demo_launch.py

Brings up Nav2 navigation (via nav2_bringup) with config/nav2_params.yaml,
which plugs ros2_dynamic_path_planning/DStarLitePlanner into the
GridBased planner slot. To compare against A*, change the `plugin:`
line in config/nav2_params.yaml to ros2_dynamic_path_planning/AStarPlanner.

Requires a map/localization source (e.g. nav2_bringup's TB3 simulation)
to be running alongside.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("ros2_dynamic_path_planning")
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": os.path.join(pkg_share, "config", "nav2_params.yaml"),
                }.items(),
            ),
        ]
    )
