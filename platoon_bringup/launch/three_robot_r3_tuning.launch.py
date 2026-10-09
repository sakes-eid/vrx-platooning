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

    follower_spacing = float(
        LaunchConfiguration(
            'follower_spacing'
        ).perform(context)
    )

    if follower_spacing <= 0.0:
        raise ValueError(
            'follower_spacing must be > 0'
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

    follower_id = LaunchConfiguration(
        'follower_id'
    ).perform(context)

    predecessor_id = LaunchConfiguration(
        'predecessor_id'
    ).perform(context)

    if follower_id != 'r3' or predecessor_id != 'r2':
        raise ValueError(
            'three_robot_r3_tuning.launch.py requires '
            'follower_id=r3 and predecessor_id=r2'
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
            'R3 tuning result already exists: '
            + follower_csv
        )

    bringup = get_package_share_directory(
        'platoon_bringup'
    )

    control = get_package_share_directory(
        'platoon_control'
    )

    planner = get_package_share_directory(
        'platoon_planner'
    )

    # ---------------------------------------------------------
    # Models
    # ---------------------------------------------------------

    r1 = Model(
        'wamv',
        'wam-v',
        [-532.0, 162.0, 0.0, 0.0, 0.0, 1.0],
    )
    r1.set_urdf(
        os.path.join(
            bringup,
            'urdf',
            'wamv_light.urdf.xacro',
        )
    )

    r2 = Model(
        'wamv2',
        'wam-v',
        [-520.0, 162.0, 0.0, 0.0, 0.0, 1.0],
    )
    r2.set_urdf(
        os.path.join(
            bringup,
            'urdf',
            'wamv_light_r2.urdf.xacro',
        )
    )

    r3 = Model(
        'wamv3',
        'wam-v',
        [-508.0, 162.0, 0.0, 0.0, 0.0, 1.0],
    )
    r3.set_urdf(
        os.path.join(
            bringup,
            'urdf',
            'wamv_light_r3.urdf.xacro',
        )
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
            [r1, r2, r3],
        )
    )

    actions.extend(
        vrx_gz.launch.competition_bridges(
            'sydney_regatta',
            False,
        )
    )

    # ---------------------------------------------------------
    # State estimators
    # ---------------------------------------------------------

    for vehicle_id, model in (
        ('r1', 'wamv'),
        ('r2', 'wamv2'),
        ('r3', 'wamv3'),
    ):
        actions.append(
            Node(
                package='platoon_state',
                executable='multi_vehicle_state',
                namespace=vehicle_id,
                name='multi_vehicle_state',
                output='screen',
                parameters=[{
                    'vehicle_id': vehicle_id,
                    'gps_topic':
                        f'/{model}/sensors/gps/gps/fix',
                    'imu_topic':
                        f'/{model}/sensors/imu/imu/data',
                    'state_topic':
                        f'/{vehicle_id}/vehicle_state',
                    'use_sim_time': True,
                }],
            )
        )

    # ---------------------------------------------------------
    # R1 — frozen leader
    # ---------------------------------------------------------

    actions.append(
        Node(
            package='platoon_planner',
            executable='stress_course_planner',
            name='trajectory_planner',
            output='screen',
            parameters=[
                os.path.join(
                    planner,
                    'config',
                    'planner.yaml',
                ),
                {
                    'use_sim_time': True,
                },
            ],
        )
    )

    actions.append(
        Node(
            package='platoon_control',
            executable='leader_stress_controller',
            name='leader_pid_controller',
            output='screen',
            parameters=[
                os.path.join(
                    control,
                    'config',
                    'leader_pid.yaml',
                ),
                {
                    'use_sim_time': True,
                },
            ],
        )
    )

    # ---------------------------------------------------------
    # R2 <- R1
    # Frozen final V2.2 gains.
    # HOLD instead of parking in three-robot mode.
    # ---------------------------------------------------------

    actions.append(
        Node(
            package='platoon_planner',
            executable='proactive_follower_planner_v22',
            namespace='r2',
            name='follower_planner',
            output='screen',
            parameters=[
                {
                    'follower_id': 'r2',
                    'predecessor_id': 'r1',

                    'predecessor_success_topic':
                        '/r1/success',

                    'reference_path_topic':
                        '/planner/reference_path',

                    'terminal_handoff_mode':
                        'reference_path',

                    'terminal_behavior':
                        'hold',

                    'mission_state_topic':
                        '/r2/mission_state',

                    'mission_success_topic':
                        '/r2/pair_success',

                    'formation_distance':
                        follower_spacing,

                    'use_sim_time':
                        True,
                },

                os.path.join(
                    planner,
                    'config',
                    'r2_follower_v22_tuned.yaml',
                ),
            ],
        )
    )

    actions.append(
        Node(
            package='platoon_control',
            executable='proactive_follower_controller_v22',
            namespace='r2',
            name='follower_controller',
            output='screen',
            parameters=[
                {
                    'follower_id': 'r2',
                    'predecessor_id': 'r1',

                    'actuator_prefix':
                        '/wamv2',

                    'mission_state_topic':
                        '/r2/mission_state',

                    'formation_distance':
                        follower_spacing,

                    'use_sim_time':
                        True,
                },

                os.path.join(
                    control,
                    'config',
                    'r2_follower_v22_tuned.yaml',
                ),
            ],
        )
    )

    # ---------------------------------------------------------
    # R3 <- R2
    # Seeded from final R2 V2.2 gains.
    #
    # IMPORTANT:
    # No /planner/reference_path subscription.
    # Terminal handoff comes only from R2 mission state.
    # ---------------------------------------------------------

    actions.append(
        Node(
            package='platoon_planner',
            executable='proactive_follower_planner_v22',
            namespace='r3',
            name='follower_planner',
            output='screen',
            parameters=[
                {
                    'follower_id': 'r3',
                    'predecessor_id': 'r2',

                    'predecessor_success_topic':
                        '/r2/pair_success',

                    'terminal_handoff_mode':
                        'predecessor_mission',

                    'predecessor_mission_state_topic':
                        '/r2/mission_state',

                    # R3 starts when R2 is released. Its initial
                    # breadcrumb history is bootstrapped only from
                    # the local R2/R3 pair geometry.
                    'release_mode':
                        'predecessor_release_bootstrap',

                    'predecessor_release_topic':
                        '/planner/r2/released',

                    'terminal_behavior':
                        'hold',

                    'mission_state_topic':
                        '/r3/mission_state',

                    'mission_success_topic':
                        '/r3/pair_success',

                    'formation_distance':
                        follower_spacing,

                    'use_sim_time':
                        True,
                },

                planner_params_file,
            ],
        )
    )

    actions.append(
        Node(
            package='platoon_control',
            executable='proactive_follower_controller_v22',
            namespace='r3',
            name='follower_controller',
            output='screen',
            parameters=[
                {
                    'follower_id': 'r3',
                    'predecessor_id': 'r2',

                    'actuator_prefix':
                        '/wamv3',

                    'mission_state_topic':
                        '/r3/mission_state',

                    'formation_distance':
                        follower_spacing,

                    'use_sim_time':
                        True,
                },

                controller_params_file,
            ],
        )
    )

    # ---------------------------------------------------------
    # Tuning loggers
    # ---------------------------------------------------------

    # R1 logger is retained as a regression check that the frozen
    # leader still completes the same stress course.
    actions.append(
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
        )
    )

    # Only R3 is scored/tuned. R2 remains completely frozen.
    actions.append(
        Node(
            package='platoon_monitor',
            executable='follower_logger_v22',
            namespace='r3',
            name='follower_logger',
            output='screen',
            parameters=[{
                'follower_id':
                    'r3',

                'predecessor_id':
                    'r2',

                'predecessor_actuator_prefix':
                    '/wamv2',

                'follower_actuator_prefix':
                    '/wamv3',

                'mission_state_topic':
                    '/r3/mission_state',

                'predecessor_success_topic':
                    '/r2/pair_success',

                'pair_success_topic':
                    '/r3/pair_success',

                'formation_distance':
                    follower_spacing,

                'output_file':
                    follower_csv,

                'use_sim_time':
                    True,
            }],
        )
    )

    print(
        '\n'
        '============================================\n'
        ' R3 ADAPTIVE TUNING - THREE ROBOTS\n'
        '============================================\n'
        ' R1 : FROZEN leader\n'
        ' R2 : FROZEN final V2.2 follower\n'
        ' R3 : TUNABLE follower\n'
        '\n'
        ' Chain : R1 -> R2 -> R3\n'
        ' R3 global path access : NONE\n'
        ' R3 release : synchronized with R2\n'
        f' Run : {run_name}\n'
        f' CSV : {follower_csv}\n'
        '============================================\n'
    )

    return actions


def generate_launch_description():

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

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=':'.join(paths),
        ),

        DeclareLaunchArgument(
            'follower_spacing',
            default_value='5.0',
            description=(
                'Desired inter-robot formation distance in metres'
            ),
        ),

        DeclareLaunchArgument(
            'headless',
            default_value='True',
        ),

        DeclareLaunchArgument(
            'run_name',
            default_value='r3_seed_validation_01',
        ),

        DeclareLaunchArgument(
            'results_root',
            default_value=(
                '~/vrx_ws/src/vrx_platooning/'
                'results/r3_tuning'
            ),
        ),

        DeclareLaunchArgument(
            'controller_params_file',
            default_value=os.path.join(
                get_package_share_directory(
                    'platoon_control'
                ),
                'config',
                'r3_follower_v22_seed.yaml',
            ),
        ),

        DeclareLaunchArgument(
            'planner_params_file',
            default_value=os.path.join(
                get_package_share_directory(
                    'platoon_planner'
                ),
                'config',
                'r3_follower_v22_seed.yaml',
            ),
        ),

        DeclareLaunchArgument(
            'follower_id',
            default_value='r3',
        ),

        DeclareLaunchArgument(
            'predecessor_id',
            default_value='r2',
        ),

        OpaqueFunction(
            function=launch_system,
        ),
    ])
