from setuptools import find_packages, setup

package_name = 'platoon_monitor'

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
            'share/' + package_name + '/data',
            [
                'data/sydney_local_occupancy.npy'
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
        'Monitoring and visualization tools '
        'for the VRX platooning project'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [
            'path_visualizer = '
            'platoon_monitor.path_visualizer:main',

            'live_map = '
            'platoon_monitor.live_map:main',

            'trajectory_logger = '
            'platoon_monitor.trajectory_logger:main',
        
            'two_robot_logger = platoon_monitor.two_robot_logger:main',
        ],
    },
)
