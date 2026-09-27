#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('platoon_bringup')

    core_launch = os.path.join(
        bringup_share,
        'launch',
        'leader_stress_core.launch.py',
    )

    # The repository is intentionally kept in ~/vrx_ws in this project.
    # The source-side RViz config avoids changing package data_files solely
    # for a visualization file.
    rviz_config = os.path.expanduser(
        '~/vrx_ws/src/vrx_platooning/platoon_bringup/rviz/r1_stress.rviz'
    )

    run_name = LaunchConfiguration('run_name')
    headless = LaunchConfiguration('headless')
    show_map = LaunchConfiguration('show_map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_name',
            default_value='r1_stress_baseline_01',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
        ),
        DeclareLaunchArgument(
            'show_map',
            default_value='True',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'run_name': run_name,
                'headless': headless,
            }.items(),
        ),

        Node(
            package='platoon_monitor',
            executable='r1_stress_viz',
            name='r1_stress_viz',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='platoon_monitor',
            executable='r1_stress_logger',
            name='r1_stress_logger',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'run_name': run_name},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='r1_stress_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(show_map),
        ),
    ])
