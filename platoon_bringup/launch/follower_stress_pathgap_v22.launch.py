import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import vrx_gz.launch
from vrx_gz.model import Model


def launch_system(context):

    follower_id = LaunchConfiguration(
        'follower_id'
    ).perform(context)

    predecessor_id = LaunchConfiguration(
        'predecessor_id'
    ).perform(context)

    controller_params_file = os.path.expanduser(
        LaunchConfiguration(
            'controller_params_file'
        ).perform(context)
    )

    planner_params_file = os.path.expanduser(
        LaunchConfiguration(
            'planner_params_file'
        ).perform(context)
    )

    headless = (
        LaunchConfiguration('headless')
        .perform(context)
        .lower()
        == 'true'
    )

    run_name = LaunchConfiguration(
        'run_name'
    ).perform(context)

    results_root = os.path.expanduser(
        LaunchConfiguration(
            'results_root'
        ).perform(context)
    )

    os.makedirs(
        results_root,
        exist_ok=True,
    )

    follower_csv = os.path.join(
        results_root,
        run_name + '.csv',
    )

    if os.path.exists(follower_csv):
        raise RuntimeError(
            'Baseline result already exists: '
            + follower_csv
        )

    # =========================================================
    # LIGHT WAM-V models
    # =========================================================

    bringup_share = get_package_share_directory(
        'platoon_bringup'
    )

    r1_urdf = os.path.join(
        bringup_share,
        'urdf',
        'wamv_light.urdf.xacro',
    )

    r2_urdf = os.path.join(
        bringup_share,
        'urdf',
        'wamv_light_r2.urdf.xacro',
    )

    # R1 = normal validated spawn.
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
    r1.set_urdf(r1_urdf)

    # R2 starts ~12 m behind R1 along the first straight.
    # This gives a reasonable initial catch-up transient.
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
    r2.set_urdf(r2_urdf)

    actions = []

    # =========================================================
    # Gazebo / VRX
    # =========================================================

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
            [r1, r2],
        )
    )

    actions.extend(
        vrx_gz.launch.competition_bridges(
            'sydney_regatta',
            False,
        )
    )

    # =========================================================
    # Frozen R1 configuration
    # =========================================================

    leader_config = os.path.join(
        get_package_share_directory(
            'platoon_control'
        ),
        'config',
        'leader_pid.yaml',
    )

    planner_config = os.path.join(
        get_package_share_directory(
            'platoon_planner'
        ),
        'config',
        'planner.yaml',
    )

    # =========================================================
    # R1 + R2 state estimators
    # =========================================================

    actions.extend([

        Node(
            package='platoon_state',
            executable='multi_vehicle_state',
            namespace='r1',
            name='multi_vehicle_state',
            output='screen',
            parameters=[{
                'vehicle_id': 'r1',
                'gps_topic':
                    '/wamv/sensors/gps/gps/fix',
                'imu_topic':
                    '/wamv/sensors/imu/imu/data',
                'state_topic':
                    '/r1/vehicle_state',
                'use_sim_time': True,
            }],
        ),

        Node(
            package='platoon_state',
            executable='multi_vehicle_state',
            namespace='r2',
            name='multi_vehicle_state',
            output='screen',
            parameters=[{
                'vehicle_id': 'r2',
                'gps_topic':
                    '/wamv2/sensors/gps/gps/fix',
                'imu_topic':
                    '/wamv2/sensors/imu/imu/data',
                'state_topic':
                    '/r2/vehicle_state',
                'use_sim_time': True,
            }],
        ),

        # =====================================================
        # FROZEN R1 stress-course planner
        # =====================================================

        Node(
            package='platoon_planner',
            executable='stress_course_planner',
            name='trajectory_planner',
            output='screen',
            parameters=[
                planner_config,
                {
                    'use_sim_time': True,
                },
            ],
        ),

        # =====================================================
        # FROZEN R1 controller
        # =====================================================

        Node(
            package='platoon_control',
            executable='leader_stress_controller',
            name='leader_pid_controller',
            output='screen',
            parameters=[
                leader_config,
                {
                    'use_sim_time': True,
                },
            ],
        ),

        # =====================================================
        # GENERIC FOLLOWER PLANNER
        # R2 <- R1
        # =====================================================

        Node(
            package='platoon_planner',
            executable='proactive_follower_planner_v22',
            namespace='r2',
            name='follower_planner',
            output='screen',
            parameters=[{
                'follower_id': follower_id,
                'predecessor_id': predecessor_id,

                'predecessor_success_topic':
                    '/r1/success',

                'reference_path_topic':
                    '/planner/reference_path',

                'mission_state_topic':
                    '/r2/mission_state',

                'mission_success_topic':
                    '/r2/pair_success',

                'formation_distance':
                    5.0,

                'use_sim_time':
                    True,
            }, planner_params_file],
        ),

        # =====================================================
        # GENERIC FOLLOWER CONTROLLER
        # Existing R2 gains = BASELINE
        # =====================================================

        Node(
            package='platoon_control',
            executable='proactive_follower_controller_v22',
            namespace='r2',
            name='follower_controller',
            output='screen',
            parameters=[{
                'follower_id': follower_id,
                'predecessor_id': predecessor_id,

                'actuator_prefix':
                    '/wamv2',

                'mission_state_topic':
                    '/r2/mission_state',

                'formation_distance':
                    5.0,

                'use_sim_time':
                    True,
            }, controller_params_file],
        ),

        # =====================================================
        # R1 stress logger
        # Confirms frozen R1 behavior did not regress
        # =====================================================

        Node(
            package='platoon_monitor',
            executable='r1_stress_logger',
            name='r1_stress_logger',
            output='screen',
            parameters=[{
                'run_name':
                    run_name + '_r1',

                'results_root':
                    results_root,

                'use_sim_time':
                    True,
            }],
        ),

        # =====================================================
        # Generic R2 follower logger
        # =====================================================

        Node(
            package='platoon_monitor',
            executable='follower_logger_v22',
            namespace='r2',
            name='follower_logger',
            output='screen',
            parameters=[{
                'follower_id': follower_id,

                'predecessor_id': predecessor_id,

                'predecessor_actuator_prefix':
                    '/wamv',

                'follower_actuator_prefix':
                    '/wamv2',

                'mission_state_topic':
                    '/r2/mission_state',

                'predecessor_success_topic':
                    '/r1/success',

                'pair_success_topic':
                    '/r2/pair_success',

                'formation_distance':
                    5.0,

                'output_file':
                    follower_csv,

                'use_sim_time':
                    True,
            }],
        ),
    ])

    print(
        '\n'
        '============================================\n'
        ' R2 FOLLOWER STRESS BASELINE\n'
        '============================================\n'
        ' Predecessor : r1\n'
        ' Follower    : r2\n'
        ' R1 control  : FROZEN\n'
        ' R2 tuning   : NONE - current gains\n'
        ' Gap target  : 5.0 m along-path bumper gap\n'
        ' Course      : FULL STRESS COURSE\n'
        f' Run         : {run_name}\n'
        f' Output      : {follower_csv}\n'
        '============================================\n'
    )

    return actions


def generate_launch_description():

    controller_params_file = LaunchConfiguration(
        'controller_params_file'
    )
    planner_params_file = LaunchConfiguration(
        'planner_params_file'
    )

    follower_id = LaunchConfiguration('follower_id')
    predecessor_id = LaunchConfiguration('predecessor_id')


    # Gazebo resolves WAM-V meshes relative to the ROS share roots,
    # not only to each individual package directory.
    paths = [
        os.path.join(
            get_package_prefix('wamv_description'),
            'share',
        ),
        os.path.join(
            get_package_prefix('wamv_gazebo'),
            'share',
        ),
        os.path.join(
            get_package_prefix('vrx_gz'),
            'share',
        ),
        os.path.join(
            get_package_prefix('platoon_bringup'),
            'share',
        ),
    ]

    existing = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    if existing:
        paths.append(existing)

    return LaunchDescription([

        DeclareLaunchArgument(
            'follower_id',
            default_value='r2',
            description='Vehicle being tuned',
        ),

        DeclareLaunchArgument(
            'predecessor_id',
            default_value='r1',
            description='Vehicle followed by follower_id',
        ),

        DeclareLaunchArgument(
            'controller_params_file',
            description='Follower controller trial YAML',
        ),

        DeclareLaunchArgument(
            'planner_params_file',
            description='Follower planner trial YAML',
        ),


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
            default_value='v21_speed_preview_diag_01',
        ),

        DeclareLaunchArgument(
            'results_root',
            default_value=(
                '~/vrx_ws/src/vrx_platooning/'
                'results/follower_baseline'
            ),
        ),

        OpaqueFunction(
            function=launch_system
        ),
    ])
