import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    bringup_share = get_package_share_directory(
        'platoon_bringup'
    )

    core_launch = os.path.join(
        bringup_share,
        'launch',
        'follower_stress_pathgap_v22.launch.py',
    )

    # Use source-tree RViz config, same approach as the working R1 launch.
    rviz_config = os.path.expanduser(
        '~/vrx_ws/src/vrx_platooning/'
        'platoon_bringup/rviz/r1_r2_stress.rviz'
    )

    follower_id = LaunchConfiguration('follower_id')
    follower_spacing = LaunchConfiguration(
        'follower_spacing'
    )
    follower_max_speed = LaunchConfiguration(
        'follower_max_speed'
    )
    predecessor_id = LaunchConfiguration('predecessor_id')

    controller_params_file = LaunchConfiguration(
        'controller_params_file'
    )
    planner_params_file = LaunchConfiguration(
        'planner_params_file'
    )

    enable_logging = LaunchConfiguration(
        'enable_logging'
    )
    run_name = LaunchConfiguration('run_name')
    results_root = LaunchConfiguration('results_root')
    enable_planner = LaunchConfiguration(
        'enable_planner'
    )
    headless = LaunchConfiguration('headless')
    show_map = LaunchConfiguration('show_map')
    show_rviz = LaunchConfiguration('show_rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'follower_spacing',
            default_value='5.0',
        ),

        DeclareLaunchArgument(
            'follower_max_speed',
            default_value='2.8',
        ),

        DeclareLaunchArgument(
            'follower_id',
            default_value='r2',
        ),

        DeclareLaunchArgument(
            'predecessor_id',
            default_value='r1',
        ),

        DeclareLaunchArgument(
            'controller_params_file',
        ),

        DeclareLaunchArgument(
            'planner_params_file',
        ),

        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='Enable result logging',
        ),

        DeclareLaunchArgument(
            'run_name',
            default_value='r2_best_visual_01',
        ),

        DeclareLaunchArgument(
            'results_root',
            default_value=os.path.expanduser(
                '~/vrx_ws/src/vrx_platooning/'
                'results/manual_validation'
            ),
        ),

        DeclareLaunchArgument(
            'enable_planner',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'headless',
            default_value='False',
        ),

        DeclareLaunchArgument(
            'show_map',
            default_value='True',
        ),
        DeclareLaunchArgument(
            'show_rviz',
            default_value='True',
        ),

        # Existing proven R1 + R2 simulation/control stack.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'follower_id': follower_id,
                'follower_spacing':
                    follower_spacing,
                'follower_max_speed':
                    follower_max_speed,
                'predecessor_id': predecessor_id,
                'controller_params_file':
                    controller_params_file,
                'planner_params_file':
                    planner_params_file,
                'enable_logging':
                    enable_logging,
                'run_name': run_name,
                'results_root': results_root,
                'enable_planner':
                    enable_planner,
                'headless': headless,
            }.items(),
        ),

        # Same RViz frame bridge used by the working R1 visual launch.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_ned_rviz_anchor_tf',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'world_ned',
                '--child-frame-id', 'rviz_anchor',
            ],
            condition=IfCondition(show_rviz),
        ),

        # R1 visualization.
        Node(
            package='platoon_monitor',
            executable='r1_stress_viz',
            name='r1_stress_viz',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            condition=IfCondition(show_rviz),
        ),

        # R2 visualization.
        Node(
            package='platoon_monitor',
            executable='r2_stress_viz',
            name='r2_stress_viz',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
            condition=IfCondition(show_rviz),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='r1_r2_stress_rviz',
            output='screen',
            arguments=[
                '-d',
                rviz_config,
            ],
            condition=IfCondition(show_rviz),
        ),
    ])
