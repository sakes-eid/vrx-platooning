from setuptools import find_packages, setup

package_name = 'platoon_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/config',
            ['config/planner.yaml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sajed',
    maintainer_email='sajed@example.com',
    description=(
        'Reference trajectory planner for the '
        'VRX platooning project.'
    ),
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_planner = '
            'platoon_planner.trajectory_planner:main',
        
            'coverage_planner = platoon_planner.coverage_planner:main',
        
            'two_robot_planner = platoon_planner.two_robot_planner:main',
        
            'long_straight_planner = platoon_planner.long_straight_planner:main',
        ],
    },
)