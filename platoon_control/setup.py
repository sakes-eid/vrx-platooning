from setuptools import find_packages, setup

package_name = 'platoon_control'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ]
        ),
        (
            'share/' + package_name,
            [
                'package.xml'
            ]
        ),
        (
            'share/' + package_name + '/config',
            [
                'config/leader_pid.yaml'
            ]
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='sajed',

    maintainer_email='sajed@example.com',

    description=(
        'PID trajectory control '
        'for the VRX platooning project'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [
            'leader_stress_controller = platoon_control.leader_stress_controller:main',
            'leader_pid_controller = '
            'platoon_control.leader_pid_controller:main',
        
            'follower_pid_controller = platoon_control.follower_pid_controller:main',
            'follower_controller = platoon_control.follower_pid_controller:main',
            'proactive_follower_controller = platoon_control.proactive_follower_controller:main',
            'proactive_follower_controller_v21 = platoon_control.proactive_follower_controller_v21:main',
            'proactive_follower_controller_v22 = platoon_control.proactive_follower_controller_v22:main',
        
            'leader_thrust_gate = platoon_control.leader_thrust_gate:main',
        ],
    },
)
