#!/usr/bin/env python3
"""
launch/astar_launch.py
Full Nav2 stack with your custom A* global planner
No hard-coded package name anywhere so anyone can use it without crashing (You're welcome ;))
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
        # Let user (or another launch file) specify the package name
        DeclareLaunchArgument(
            'pkg_name',
            default_value=TextSubstitution(text='ros2_astar_dstar_lite'),
            description='Name of your package (change if you renamed the repo)'
        ),

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value='maps/indoor_grid.yaml'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),

        # Nav2 bringup with FULL dynamic path resolution
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
                    # Force your A* planner
                    'planner_server.global_planner': TextSubstitution(text='astar_planner'),
                    'astar_planner.heuristic_weight': TextSubstitution(text='1.2'),
                    'astar_planner.allow_diagonal': TextSubstitution(text='true'),
                }.items()
            ),

            # RViz
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', [FindPackageShare(LaunchConfiguration('pkg_name')), '/rviz/nav2_config.rviz']],
                condition=IfCondition(LaunchConfiguration('rviz'))
            ),
        ])
    ])
