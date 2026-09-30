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

    r3_urdf = os.path.join(
        bringup_share,
        'urdf',
        'wamv_light_r3.urdf.xacro',
    )

    r1 = Model(
        'wamv',
        'wam-v',
        [-532.0, 162.0, 0.0, 0.0, 0.0, 1.0],
    )
    r1.set_urdf(r1_urdf)

    r2 = Model(
        'wamv2',
        'wam-v',
        [-520.0, 162.0, 0.0, 0.0, 0.0, 1.0],
    )
    r2.set_urdf(r2_urdf)

    r3 = Model(
        'wamv3',
        'wam-v',
        [-508.0, 162.0, 0.0, 0.0, 0.0, 1.0],
    )
    r3.set_urdf(r3_urdf)

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

    # State estimation is included now so the smoke test also
    # verifies GPS/IMU namespacing for all three vehicles.
    actions.extend([

        Node(
            package='platoon_state',
            executable='multi_vehicle_state',
            namespace='r1',
            name='multi_vehicle_state',
            output='screen',
            parameters=[{
                'vehicle_id': 'r1',
                'gps_topic': '/wamv/sensors/gps/gps/fix',
                'imu_topic': '/wamv/sensors/imu/imu/data',
                'state_topic': '/r1/vehicle_state',
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
                'gps_topic': '/wamv2/sensors/gps/gps/fix',
                'imu_topic': '/wamv2/sensors/imu/imu/data',
                'state_topic': '/r2/vehicle_state',
                'use_sim_time': True,
            }],
        ),

        Node(
            package='platoon_state',
            executable='multi_vehicle_state',
            namespace='r3',
            name='multi_vehicle_state',
            output='screen',
            parameters=[{
                'vehicle_id': 'r3',
                'gps_topic': '/wamv3/sensors/gps/gps/fix',
                'imu_topic': '/wamv3/sensors/imu/imu/data',
                'state_topic': '/r3/vehicle_state',
                'use_sim_time': True,
            }],
        ),
    ])

    print(
        '\n'
        '============================================\n'
        ' THREE-WAM-V SMOKE TEST\n'
        '============================================\n'
        ' R1 : wamv  @ (-532, 162)\n'
        ' R2 : wamv2 @ (-520, 162)\n'
        ' R3 : wamv3 @ (-508, 162)\n'
        ' Controllers : OFF\n'
        ' Purpose     : mesh/spawn/sensor validation\n'
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
            function=launch_system
        ),
    ])
