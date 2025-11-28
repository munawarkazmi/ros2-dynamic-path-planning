#!/usr/bin/env python3
"""
launch/dstar_lite_launch.py
Full Nav2 stack with your custom D* Lite global planner
Completely future-proof — no hard-coded paths or package names
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # Let user override package name (e.g. after forking/renaming)
        DeclareLaunchArgument(
            'pkg_name',
            default_value=TextSubstitution(text='ros2_astar_dstar_lite'),
            description='Name of your package (change if renamed)'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='maps/indoor_grid.yaml',
            description='Relative path to map YAML file'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz with pre-configured view'
        ),

        # Full Nav2 bringup — dynamically resolved paths
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('nav2_bringup'), '/launch/navigation_launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'map': [FindPackageShare(LaunchConfiguration('pkg_name')), '/', LaunchConfiguration('map')],
                    'params_file': [FindPackageShare(LaunchConfiguration('pkg_name')), '/config/nav2_params.yaml'],
                    # Only difference from astar_launch.py is this line (for anyone wondering)
                    'planner_server.global_planner': TextSubstitution(text='dstar_lite_planner'),
                    'dstar_lite_planner.heuristic_weight': TextSubstitution(text='1.0'),
                    'dstar_lite_planner.allow_diagonal': TextSubstitution(text='true'),
                    'dstar_lite_planner.replan_on_costmap_change': TextSubstitution(text='true'),
                }.items()
            ),

            # Optional: RViz with config
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d', [FindPackageShare(LaunchConfiguration('pkg_name')), '/rviz/nav2_config.rviz']
                ],
                condition=IfCondition(LaunchConfiguration('rviz'))
            ),
        ])
    ])
