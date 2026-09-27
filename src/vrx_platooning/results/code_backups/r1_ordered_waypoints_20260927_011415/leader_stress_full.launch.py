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

        # Reuse the user's already-working R1-only VRX launch for Gazebo,
        # WAM-V spawn, bridges, stress planner and stress controller.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'run_name': run_name,
                'headless': headless,
            }.items(),
        ),

        # IMPORTANT:
        # The original single-R1 launch uses the older state pipeline and does
        # not create the /r1/vehicle_state topic expected by the new stress
        # planner/controller/map.  Start the generic state publisher explicitly.
        #
        # Same physical R1 sensors; absolute world_ned output.
        Node(
            package='platoon_state',
            executable='multi_vehicle_state',
            namespace='r1',
            name='multi_vehicle_state_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'vehicle_id': 'r1'},
                {'gps_topic': '/wamv/sensors/gps/gps/fix'},
                {'imu_topic': '/wamv/sensors/imu/imu/data'},
                {'state_topic': '/r1/vehicle_state'},
            ],
        ),

        # Visualization bridge and RViz are baseline-only.  They are both
        # completely absent from later headless tuning when show_map:=False.
        Node(
            package='platoon_monitor',
            executable='r1_stress_viz',
            name='r1_stress_viz',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(show_map),
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
