import os

from glob import glob

from setuptools import (
    find_packages,
    setup,
)


package_name = (
    'platoon_bringup'
)


setup(

    name=package_name,

    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[

(
    os.path.join(
        'share',
        package_name,
        'urdf'
    ),
    glob(
        'urdf/*.xacro'
    )
),
(
    os.path.join(
        'share',
        package_name,
        'urdf',
        'components'
  	  ),
    glob(
        'urdf/components/*.xacro'
    )
),

        (
            'share/ament_index/'
            'resource_index/packages',

            [
                'resource/'
                + package_name
            ]
        ),

        (
            'share/'
            + package_name,

            [
                'package.xml'
            ]
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),

            glob(
                'launch/*.launch.py'
            )
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='sajed',

    maintainer_email=(
        'sajed@example.com'
    ),

    description=(
        'Launch and experiment configuration '
        'for the VRX platooning project'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [

            'experiment_launcher = '
            'platoon_bringup.'
            'experiment_launcher:main',
        ],
    },
)
