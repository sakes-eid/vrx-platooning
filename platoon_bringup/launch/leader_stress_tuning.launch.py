#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def launch_trial(context):
    run_name = LaunchConfiguration('run_name').perform(context)
    controller_params = LaunchConfiguration('controller_params_file')
    planner_params = LaunchConfiguration('planner_params_file')
    results_root = os.path.expanduser(
        LaunchConfiguration('results_root').perform(context)
    )

    os.makedirs(results_root, exist_ok=True)

    robot_urdf = os.path.join(
        get_package_share_directory('platoon_bringup'),
        'urdf',
        'wamv_light.urdf.xacro',
    )

    vrx_launch_file = os.path.join(
        get_package_share_directory('vrx_gz'),
        'launch',
        'competition.launch.py',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vrx_launch_file),
        launch_arguments={
            'world': 'sydney_regatta',
            'urdf': robot_urdf,
            'headless': 'True',
            'paused': 'False',
        }.items(),
    )

    state = Node(
        package='platoon_state',
        executable='multi_vehicle_state',
        namespace='r1',
        name='multi_vehicle_state_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'vehicle_id': 'r1',
            'gps_topic': '/wamv/sensors/gps/gps/fix',
            'imu_topic': '/wamv/sensors/imu/imu/data',
            'state_topic': '/r1/vehicle_state',
        }],
    )

    planner = Node(
        package='platoon_planner',
        executable='stress_course_planner',
        name='trajectory_planner',
        output='screen',
        parameters=[
            ParameterFile(planner_params, allow_substs=True),
            {'use_sim_time': True},
        ],
    )

    controller = Node(
        package='platoon_control',
        executable='leader_stress_controller',
        name='leader_pid_controller',
        output='screen',
        parameters=[
            ParameterFile(controller_params, allow_substs=True),
            {'use_sim_time': True},
        ],
    )

    logger = Node(
        package='platoon_monitor',
        executable='r1_stress_logger',
        name='r1_stress_logger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'run_name': run_name,
            'results_root': results_root,
        }],
    )

    return [gazebo, state, planner, controller, logger]


def generate_launch_description():
    resources = [
        os.path.join(get_package_prefix('wamv_description'), 'share'),
        os.path.join(get_package_prefix('wamv_gazebo'), 'share'),
        os.path.join(get_package_prefix('vrx_gz'), 'share'),
        os.path.join(get_package_prefix('platoon_bringup'), 'share'),
    ]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        resources.append(existing)

    return LaunchDescription([
        DeclareLaunchArgument('run_name'),
        DeclareLaunchArgument('controller_params_file'),
        DeclareLaunchArgument('planner_params_file'),
        DeclareLaunchArgument(
            'results_root',
            default_value=os.path.expanduser(
                '~/vrx_ws/src/vrx_platooning/results/autotune_r1_stress/trials'
            ),
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=':'.join(resources),
        ),
        OpaqueFunction(function=launch_trial),
    ])
