import os

from datetime import datetime


def ask_yes_no(
    question,
    default=False
):

    if default:
        prompt = ' [Y/n]: '
    else:
        prompt = ' [y/N]: '

    while True:

        answer = input(
            question + prompt
        ).strip().lower()

        if answer == '':

            return default

        if answer in (
            'y',
            'yes',
        ):

            return True

        if answer in (
            'n',
            'no',
        ):

            return False

        print(
            'Please enter y or n.'
        )


def ask_trajectory():

    while True:

        answer = input(
            'Trajectory '
            '[straight/curved] '
            '[straight]: '
        ).strip().lower()

        if answer == '':

            return 'straight'

        if answer in (
            'straight',
            'curved',
        ):

            return answer

        print(
            'Choose straight or curved.'
        )


def ask_simulation_profile():

    print(
        '\n'
        'Simulation profile:\n'
        '  1. Light  - GPS + IMU, cameras/lidar disabled\n'
        '  2. Full   - original VRX sensor configuration\n'
    )

    while True:

        answer = input(
            'Profile [1]: '
        ).strip().lower()

        if answer in (
            '',
            '1',
            'light',
            'l',
        ):

            return 'light'

        if answer in (
            '2',
            'full',
            'f',
        ):

            return 'full'

        print(
            'Choose 1/light or 2/full.'
        )


def main():

    print(
        '\n'
        '============================================\n'
        ' VRX PLATOONING EXPERIMENT SETUP\n'
        '============================================'
    )

    # =========================================================
    # Simulation profile
    # =========================================================

    simulation_profile = (
        ask_simulation_profile()
    )

    # =========================================================
    # Trajectory
    # =========================================================

    trajectory = (
        ask_trajectory()
    )

    # =========================================================
    # Live map
    # =========================================================

    show_live_map = (
        ask_yes_no(
            'Display live map?',
            default=False
        )
    )

    # =========================================================
    # Unique default run name
    # =========================================================

    timestamp = (
        datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )
    )

    default_run_name = (
        f'{trajectory}_'
        f'{simulation_profile}_'
        f'{timestamp}'
    )

    run_name = input(
        f'Run name '
        f'[{default_run_name}]: '
    ).strip()

    if run_name == '':

        run_name = (
            default_run_name
        )

    # =========================================================
    # Convert options for ROS launch
    # =========================================================

    live_map_value = (

        'true'

        if show_live_map

        else 'false'
    )

    # =========================================================
    # Summary
    # =========================================================

    print(
        '\n'
        '============================================\n'
        ' STARTING EXPERIMENT\n'
        '============================================\n'
        f' Profile    : {simulation_profile}\n'
        f' Trajectory : {trajectory}\n'
        f' Live map   : {show_live_map}\n'
        f' Run name   : {run_name}\n'
        '============================================\n'
    )

    # =========================================================
    # Launch
    # =========================================================

    command = [

        'ros2',

        'launch',

        'platoon_bringup',

        'leader_tracking.launch.py',

        (
            f'simulation_profile:='
            f'{simulation_profile}'
        ),

        (
            f'trajectory_type:='
            f'{trajectory}'
        ),

        (
            f'run_name:='
            f'{run_name}'
        ),

        (
            f'show_live_map:='
            f'{live_map_value}'
        ),
    ]

    os.execvp(
        command[0],
        command
    )


if __name__ == '__main__':

    main()