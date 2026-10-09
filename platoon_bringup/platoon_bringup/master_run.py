#!/usr/bin/env python3

"""
Top-level execution for VRX platooning startup.

Test mode:
    robot stack with built-in R1 planner disabled
        ->
    wait for live R1 state
        ->
    launch selected external R1 mission planner

Tuning mode:
    launch the existing selected autotuner

All child processes are cleaned up when the run ends.
"""

import os
import atexit
import time
import signal
import subprocess
import time

from platoon_bringup.robot_launch import (
    build_robot_launch_spec,
)

from platoon_bringup.tuning_launch import (
    build_tuning_launch_spec_from_config,
)


PLANNER_EXECUTABLES = {
    "straight":
        "straight_mission_planner",

    "curve":
        "curve_mission_planner",

    "coverage":
        "coverage_mission_planner",

    "stress":
        "stress_course_planner",
}


def _bool_text(value):

    return (
        "true"
        if value
        else "false"
    )


def _cleanup_gazebo():
    """
    Final safety-net cleanup.

    ros2 launch normally tears Gazebo down with its process group,
    but gz sim can occasionally survive as an orphan after Ctrl+C.
    This cleanup is intentionally limited to Gazebo processes.
    """

    patterns = (
        "gz sim",
        "gzserver",
        "gzclient",
    )

    # Graceful termination first.
    for pattern in patterns:
        subprocess.run(
            [
                "pkill",
                "-TERM",
                "-f",
                pattern,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    time.sleep(1.0)

    # Hard cleanup only for anything that survived SIGTERM.
    for pattern in patterns:
        subprocess.run(
            [
                "pkill",
                "-KILL",
                "-f",
                pattern,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


# Also run if Python leaves through Ctrl+C or a normal exception.
atexit.register(_cleanup_gazebo)


def _stop_process_group(process):

    if process is None:
        return

    if process.poll() is not None:
        return

    try:
        os.killpg(
            os.getpgid(process.pid),
            signal.SIGINT,
        )

        process.wait(
            timeout=8.0
        )

        return

    except Exception:
        pass

    try:
        os.killpg(
            os.getpgid(process.pid),
            signal.SIGTERM,
        )

        process.wait(
            timeout=4.0
        )

        return

    except Exception:
        pass

    try:
        os.killpg(
            os.getpgid(process.pid),
            signal.SIGKILL,
        )

    except Exception:
        pass


def _wait_for_r1_state(timeout_sec=45.0):
    """
    Wait until the R1 state topic exists AND has published
    at least one VehicleState message.

    `ros2 topic echo --once` may exit immediately when the
    topic does not exist yet, so startup must retry instead
    of treating that initial condition as a fatal error.
    """

    deadline = time.monotonic() + float(timeout_sec)
    last_error = ""

    print(
        "Waiting for /r1/vehicle_state..."
    )

    while time.monotonic() < deadline:

        # First wait until ROS knows the topic and its type.
        try:
            probe = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "type",
                    "/r1/vehicle_state",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3.0,
                check=False,
            )

            if probe.returncode != 0:
                last_error = (
                    probe.stderr.strip()
                    or probe.stdout.strip()
                    or "topic not available yet"
                )

                time.sleep(0.5)
                continue

        except subprocess.TimeoutExpired:
            last_error = "topic-type probe timed out"
            time.sleep(0.5)
            continue

        # Topic exists. Now require one real state message.
        remaining = deadline - time.monotonic()

        if remaining <= 0.0:
            break

        try:
            result = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "echo",
                    "/r1/vehicle_state",
                    "--once",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                timeout=min(5.0, remaining),
                check=False,
            )

            if result.returncode == 0:
                print(
                    "[PASS] /r1/vehicle_state is publishing"
                )
                return

            last_error = (
                result.stderr.strip()
                or "topic exists but no usable message yet"
            )

        except subprocess.TimeoutExpired:
            last_error = (
                "topic exists but no state message arrived yet"
            )

        time.sleep(0.5)

    message = (
        "Timed out waiting for /r1/vehicle_state "
        f"after {timeout_sec:.0f} seconds."
    )

    if last_error:
        message += "\nLast ROS status: " + last_error

    raise RuntimeError(message)


def _build_robot_command(
    config,
):

    spacing = (
        config.follower_spacing
        if config.follower_spacing
        is not None
        else 5.0
    )

    spec = build_robot_launch_spec(
        config.robot_count,

        headless=config.headless,

        show_map=config.show_map,

        run_name=config.run_name,

        simulation_profile="light",

        follower_spacing=spacing,

        leader_max_speed=config.max_speed,

        results_dir=config.results_dir,
    )

    # ---------------------------------------------------------
    # Step 6 settings that every current stack supports.
    # ---------------------------------------------------------

    spec.arguments[
        "show_rviz"
    ] = _bool_text(
        config.show_rviz
    )

    spec.arguments[
        "enable_logging"
    ] = _bool_text(
        config.logging_enabled
    )

    spec.arguments[
        "run_name"
    ] = str(
        config.run_name
    )

    # R1 calls it results_dir.
    if config.robot_count == 1:

        spec.arguments[
            "results_dir"
        ] = str(
            config.results_dir
        )

    # Follower stacks call it results_root.
    else:

        spec.arguments[
            "results_root"
        ] = str(
            config.results_dir
        )

    return spec.command()


def _run_tuning(
    config,
):

    spec = (
        build_tuning_launch_spec_from_config(
            config
        )
    )

    command = (
        spec.command()
    )

    print()
    print("=" * 60)
    print("STARTING TUNING")
    print("=" * 60)
    print(
        " ".join(command)
    )
    print("=" * 60)

    process = None

    try:

        process = subprocess.Popen(
            command,
            start_new_session=True,
        )

        return_code = (
            process.wait()
        )

        if return_code != 0:

            raise RuntimeError(
                "Tuning process exited with "
                f"code {return_code}."
            )

    except KeyboardInterrupt:

        print()
        print(
            "Stopping tuning..."
        )

    finally:

        _stop_process_group(
            process
        )


def _run_new_test_mission(
    config,
):

    if config.mission_source == "saved":

        planner = "saved_mission_planner"

    else:

        planner = (
            PLANNER_EXECUTABLES[
                config.mission_type
            ]
        )

    robot_command = (
        _build_robot_command(
            config
        )
    )

    planner_command = [
        "ros2",
        "run",
        "platoon_planner",
        planner,

        "--ros-args",

        "-p",
        (
            "max_speed:="
            f"{float(config.max_speed)}"
        ),
    ]

    if config.mission_source == "saved":

        planner_command += [
            "-p",
            (
                "mission_file:="
                f"{config.saved_mission_file}"
            ),
        ]

    robot_process = None
    planner_process = None

    print()
    print("=" * 60)
    print("STARTING ROBOT STACK")
    print("=" * 60)
    print(
        " ".join(
            robot_command
        )
    )
    print("=" * 60)

    try:

        robot_process = subprocess.Popen(
            robot_command,
            start_new_session=True,
        )

        # Give ros2 launch a brief opportunity to fail early
        # before waiting for VehicleState.
        time.sleep(
            1.5
        )

        early_return = (
            robot_process.poll()
        )

        if early_return is not None:

            raise RuntimeError(
                "Robot launch exited early with "
                f"code {early_return}."
            )

        _wait_for_r1_state()

        print()
        print("=" * 60)
        print(
            "STARTING R1 MISSION PLANNER"
        )
        print("=" * 60)
        print(
            " ".join(
                planner_command
            )
        )
        print("=" * 60)

        planner_process = (
            subprocess.Popen(
                planner_command,
                start_new_session=True,
            )
        )

        return_code = (
            planner_process.wait()
        )

        if return_code != 0:

            raise RuntimeError(
                "Mission planner exited with "
                f"code {return_code}."
            )

    except KeyboardInterrupt:

        print()
        print(
            "Stopping platoon run..."
        )

    finally:

        print()
        print(
            "Cleaning up run processes..."
        )

        _stop_process_group(
            planner_process
        )

        _stop_process_group(
            robot_process
        )

        print(
            "Run stopped cleanly."
        )


def run_configured_session(
    config,
):

    if config.run_mode == "tuning":

        return _run_tuning(
            config
        )

    return _run_new_test_mission(
        config
    )
