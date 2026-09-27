import os

from glob import glob

from setuptools import (
    find_packages,
    setup,
)


package_name = 'platoon_tuning'


setup(

    name=package_name,

    version='0.0.1',

    packages=find_packages(
        exclude=[
            'test'
        ]
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
            os.path.join(
                'share',
                package_name,
                'config',
            ),
            glob(
                'config/*.yaml'
            ),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob(
                'launch/*.launch.py'
            ),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'worlds',
                'tuning',
            ),
            glob(
                'worlds/tuning/*.sdf'
            ),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='sajed',

    maintainer_email='sajed@example.com',

    description=(
        'Automatic PID tuning framework '
        'for the VRX platooning project.'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [
            'r1_stress_autotune_v2 = platoon_tuning.r1_stress_autotune_v2:main',
            'r1_stress_autotune = platoon_tuning.r1_stress_autotune:main',

            (
                'autotune = '
                'platoon_tuning.autotune:main'
            ),

            (
                'trial_analyzer = '
                'platoon_tuning.'
                'trial_analyzer:main'
            ),

            (
                'trial_runner = '
                'platoon_tuning.'
                'trial_runner:main'
            ),

            (
                'smart_autotune = '
                'platoon_tuning.'
                'smart_autotune:main'
            ),
        ],
    },
)

