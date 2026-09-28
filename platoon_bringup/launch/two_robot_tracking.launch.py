import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)

from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node

import vrx_gz.launch

from vrx_gz.model import Model


def launch_system(context):

    headless = (
        LaunchConfiguration(
            'headless'
        ).perform(context).lower()
        == 'true'
    )

    run_name = (
        LaunchConfiguration(
            'run_name'
        ).perform(context)
    )

    results_dir = os.path.expanduser(
        LaunchConfiguration(
            'results_dir'
        ).perform(context)
    )

    os.makedirs(
        results_dir,
        exist_ok=True,
    )

    output_file = os.path.join(
        results_dir,
        run_name + '.csv',
    )

    if os.path.exists(output_file):

        raise RuntimeError(
            'Result already exists: '
            + output_file
        )

    r1_urdf = os.path.join(
        get_package_share_directory(
            'platoon_bringup'
        ),
        'urdf',
        'wamv_light.urdf.xacro',
    )

    r2_urdf = os.path.join(
        get_package_share_directory(
            'platoon_bringup'
        ),
        'urdf',
        'wamv_light_r2.urdf.xacro',
    )

    r1 = Model(
        'wamv',
        'wam-v',
        [
            -532.0,
            162.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    )

    r1.set_urdf(
        r1_urdf
    )

    r2 = Model(
        'wamv2',
        'wam-v',
        [
            -520.0,
            162.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    )

    r2.set_urdf(
        r2_urdf
    )

    actions = []

    actions.extend(
        vrx_gz.launch.simulation(
            'sydney_regatta',
            headless=headless,
            paused=False,
            extra_gz_args='',
        )
    )

    actions.extend(
        vrx_gz.launch.spawn(
            'full',
            'sydney_regatta',
            [
                r1,
                r2,
            ],
        )
    )

    actions.extend(
        vrx_gz.launch.competition_bridges(
            'sydney_regatta',
            False,
        )
    )

    state_config = os.path.join(
        get_package_share_directory(
            'platoon_state'
        ),
        'config',
        'origin.yaml',
    )

    leader_config = os.path.join(
        get_package_share_directory(
            'platoon_control'
        ),
        'config',
        'leader_pid_tuned.yaml',
    )

    actions.extend([

        # ==============================================
        # ORIGINAL R1 state
        # ==============================================

        Node(
            package='platoon_state',
            executable='vehicle_state',
            name='vehicle_state_node',
            output='screen',

            parameters=[
                state_config,
                {
                    'use_sim_time':
                        True,
                }
            ],
        ),

        # ==============================================
        # R1 100m path
        # ==============================================

        Node(
            package='platoon_planner',
            executable='long_straight_planner',
            name='long_straight_planner',
            output='screen',

            parameters=[
                {
                    'start_north':
                        163.0,

                    'start_east':
                        -531.0,

                    'end_north':
                        263.0,

                    'end_east':
                        -531.0,

                    'spacing':
                        0.5,

                    'use_sim_time':
                        True,
                }
            ],
        ),

        # ==============================================
        # ORIGINAL validated R1 controller
        # ==============================================

        Node(
            package='platoon_control',
            executable='leader_pid_controller',
            name='leader_pid_controller',
            output='screen',

            parameters=[
                leader_config,
                {
                    'use_sim_time':
                        True,
                }
            ],
        ),

        # ==============================================
        # Unique R1 platoon state
        # ==============================================

        Node(
            package='platoon_state',
            executable='multi_vehicle_state',
            namespace='r1',
            name='vehicle_state_node',
            output='screen',

            parameters=[
                {
                    'vehicle_id':
                        'r1',

                    'gps_topic':
                        '/wamv/sensors/gps/gps/fix',

                    'imu_topic':
                        '/wamv/sensors/imu/imu/data',

                    'state_topic':
                        '/r1/vehicle_state',

                    'use_sim_time':
                        True,
                }
            ],
        ),

        # ==============================================
        # Unique R2 state
        # ==============================================

        Node(
            package='platoon_state',
            executable='multi_vehicle_state',
            namespace='r2',
            name='vehicle_state_node',
            output='screen',

            parameters=[
                {
                    'vehicle_id':
                        'r2',

                    'gps_topic':
                        '/wamv2/sensors/gps/gps/fix',

                    'imu_topic':
                        '/wamv2/sensors/imu/imu/data',

                    'state_topic':
                        '/r2/vehicle_state',

                    'use_sim_time':
                        True,
                }
            ],
        ),

        # ==============================================
        # Platoon planner
        # ==============================================

        Node(
            package='platoon_planner',
            executable='two_robot_planner',
            name='two_robot_planner',
            output='screen',

            parameters=[
                {
                    'formation_distance':
                        5.0,

                    'minimum_extra_trail':
                        1.0,

                    'parking_separation':
                        18.0,

                    'parking_escape_distance':
                        10.0,

                    'use_sim_time':
                        True,
                }
            ],
        ),

        # ==============================================
        # R2
        # ==============================================

        Node(
            package='platoon_control',
            executable='follower_pid_controller',
            namespace='r2',
            name='follower_pid_controller',
            output='screen',

            parameters=[
                {
                    'distance_kp':
                        0.35,

                    'distance_ki':
                        0.02,

                    'distance_kd':
                        0.10,

                    'formation_distance':
                        5.0,

                    'catchup_distance':
                        6.0,

                    'catchup_min_speed':
                        1.20,

                    'follow_max_speed':
                        1.50,

                    'collision_warning_clearance':
                        3.0,

                    'use_sim_time':
                        True,
                }
            ],
        ),

        # ==============================================
        # LOGGER
        # ==============================================

        Node(
            package='platoon_monitor',
            executable='two_robot_logger',
            name='two_robot_logger',
            output='screen',

            parameters=[
                {
                    'output_file':
                        output_file,

                    'formation_distance':
                        5.0,

                    'use_sim_time':
                        True,
                }
            ],
        ),
    ])

    return actions


def generate_launch_description():

    paths = [
        get_package_share_directory(
            'wamv_description'
        ),

        get_package_share_directory(
            'wamv_gazebo'
        ),

        get_package_share_directory(
            'vrx_gz'
        ),
    ]

    existing = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    if existing:
        paths.append(existing)

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=':'.join(paths),
        ),

        DeclareLaunchArgument(
            'headless',
            default_value='False',
        ),

        DeclareLaunchArgument(
            'run_name',
            default_value='two_robot_100m_01',
        ),

        DeclareLaunchArgument(
            'results_dir',

            default_value=(
                '~/vrx_ws/src/vrx_platooning/'
                'results/two_robot'
            ),
        ),

        OpaqueFunction(
            function=launch_system
        ),
    ])
