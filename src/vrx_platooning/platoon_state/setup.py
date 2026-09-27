from setuptools import find_packages, setup

package_name = 'platoon_state'

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
            ['config/origin.yaml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sajed',
    maintainer_email='sajed@example.com',
    description=(
        'State estimation and GPS-to-local coordinate '
        'conversion for the VRX platooning project.'
    ),
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_to_local = platoon_state.gps_to_local:main',
            'vehicle_state = platoon_state.vehicle_state_node:main',
        
            'multi_vehicle_state = platoon_state.multi_vehicle_state:main',
        ],
    },
)
