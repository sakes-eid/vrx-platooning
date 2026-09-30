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

    headless = (
        LaunchConfiguration('headless')
        .perform(context)
        .lower()
        == 'true'
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
                        5.0,

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
                        5.0,

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
                        5.0,

                    'use_sim_time':
                        True,
                },

                os.path.join(
                    planner,
                    'config',
                    'r3_follower_v22_seed.yaml',
                ),
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
                        5.0,

                    'use_sim_time':
                        True,
                },

                os.path.join(
                    control,
                    'config',
                    'r3_follower_v22_seed.yaml',
                ),
            ],
        )
    )

    # ---------------------------------------------------------
    # RViz visualization
    # ---------------------------------------------------------

    actions.append(
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
        )
    )

    actions.append(
        Node(
            package='platoon_monitor',
            executable='r1_stress_viz',
            name='r1_stress_viz',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
        )
    )

    actions.append(
        Node(
            package='platoon_monitor',
            executable='follower_stress_viz',
            namespace='r2',
            name='stress_viz',
            output='screen',
            parameters=[{
                'vehicle_id': 'r2',
                'marker_r': 1.0,
                'marker_g': 0.45,
                'marker_b': 0.0,
                'use_sim_time': True,
            }],
        )
    )

    actions.append(
        Node(
            package='platoon_monitor',
            executable='follower_stress_viz',
            namespace='r3',
            name='stress_viz',
            output='screen',
            parameters=[{
                'vehicle_id': 'r3',
                'marker_r': 0.0,
                'marker_g': 0.45,
                'marker_b': 1.0,
                'use_sim_time': True,
            }],
        )
    )

    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='r1_r2_r3_stress_rviz',
            output='screen',
            arguments=[
                '-d',
                os.path.join(
                    bringup,
                    'rviz',
                    'r1_r2_r3_stress.rviz',
                ),
            ],
        )
    )

    print(
        '\n'
        '============================================\n'
        ' THREE-ROBOT CONTROLLED PLATOON\n'
        '============================================\n'
        ' R1 : frozen leader\n'
        ' R2 : frozen final V2.2 gains\n'
        ' R3 : seeded from final R2 gains\n'
        '\n'
        ' R2 <- R1 : 5 m bumper gap\n'
        ' R3 <- R2 : 5 m bumper gap\n'
        '\n'
        ' R2 terminal : HOLD\n'
        ' R3 terminal : HOLD\n'
        ' R3 global path access : NONE\n'
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
            'headless',
            default_value='False',
        ),

        OpaqueFunction(
            function=launch_system,
        ),
    ])
