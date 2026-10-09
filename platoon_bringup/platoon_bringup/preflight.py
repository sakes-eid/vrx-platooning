#!/usr/bin/env python3

"""
Automatic pre-launch checks for the VRX platooning system.

Checks only. Nothing is launched, killed, or modified.
"""

import os
import shutil
import subprocess
from pathlib import Path


DEFAULT_REPO_ROOT = (
    Path.home()
    / "vrx_ws"
    / "src"
    / "vrx_platooning"
)


def _writable_target(path):

    path = Path(path).expanduser()

    probe = path

    while not probe.exists():

        if probe.parent == probe:
            return False

        probe = probe.parent

    return (
        probe.is_dir()
        and os.access(probe, os.W_OK)
    )


def _stale_processes():

    pattern = (
        r"gz sim|"
        r"leader_stress_controller|"
        r"stress_course_planner|"
        r"proactive_follower_planner|"
        r"proactive_follower_controller|"
        r"multi_vehicle_state|"
        r"r1_stress_logger|"
        r"follower_logger|"
        r"rviz2"
    )

    result = subprocess.run(
        [
            "pgrep",
            "-af",
            pattern,
        ],
        capture_output=True,
        text=True,
    )

    lines = []

    for line in result.stdout.splitlines():

        if "pgrep -af" in line:
            continue

        lines.append(
            line.strip()
        )

    return lines


def run_preflight(
    config,
    *,
    repo_root=DEFAULT_REPO_ROOT,
    check_processes=True,
):

    repo_root = Path(
        repo_root
    ).expanduser()

    workspace = (
        repo_root
        .parent
        .parent
    )

    passed = []
    failed = []

    def check(condition, message):

        if condition:
            passed.append(message)
        else:
            failed.append(message)

    # ---------------------------------------------------------
    # Configuration
    # ---------------------------------------------------------

    try:
        config.validate()
        passed.append(
            "Startup configuration is valid"
        )

    except Exception as exc:
        failed.append(
            f"Startup configuration invalid: {exc}"
        )

    # Project now intentionally uses LIGHT only.
    check(
        config.simulation_profile == "light",
        "Simulation profile is light",
    )

    # ---------------------------------------------------------
    # Required commands
    # ---------------------------------------------------------

    for command in (
        "ros2",
        "gz",
        "xacro",
    ):

        check(
            shutil.which(command) is not None,
            f"Command available: {command}",
        )

    # ---------------------------------------------------------
    # Workspace
    # ---------------------------------------------------------

    check(
        repo_root.is_dir(),
        f"Repository exists: {repo_root}",
    )

    check(
        (
            workspace
            / "install"
            / "setup.bash"
        ).is_file(),
        "Workspace install/setup.bash exists",
    )

    # ---------------------------------------------------------
    # Required robot configuration
    # ---------------------------------------------------------

    if config.robot_count >= 2:

        required = (
            repo_root
            / "platoon_control"
            / "config"
            / "r2_follower_v22_tuned.yaml",

            repo_root
            / "platoon_planner"
            / "config"
            / "r2_follower_v22_tuned.yaml",
        )

        for path in required:

            check(
                path.is_file(),
                f"Required file exists: {path.name}",
            )

    # ---------------------------------------------------------
    # Saved mission
    # ---------------------------------------------------------

    if config.mission_source == "saved":

        path = Path(
            config.saved_mission_file
        ).expanduser()

        check(
            path.is_file(),
            f"Saved mission exists: {path.name}",
        )

    # ---------------------------------------------------------
    # Logging output
    # ---------------------------------------------------------

    if config.logging_enabled:

        check(
            _writable_target(
                config.results_dir
            ),
            "Results directory is writable",
        )

    # ---------------------------------------------------------
    # Tuning routing + seed files
    # ---------------------------------------------------------

    if config.run_mode == "tuning":

        try:

            from platoon_bringup.tuning_launch import (
                build_tuning_launch_spec_from_config,
            )

            spec = (
                build_tuning_launch_spec_from_config(
                    config,
                    repo_root=repo_root,
                )
            )

            check(
                bool(spec.command()),
                (
                    "Tuning route is valid: "
                    f"{config.tuning_target}"
                ),
            )

        except Exception as exc:

            failed.append(
                f"Tuning route invalid: {exc}"
            )

    # ---------------------------------------------------------
    # Stale simulation / platoon processes
    # ---------------------------------------------------------

    if check_processes:

        stale = _stale_processes()

        if stale:

            failed.append(
                "Stale Gazebo/platoon processes detected:\n"
                + "\n".join(
                    f"    {line}"
                    for line in stale
                )
            )

        else:

            passed.append(
                "No stale Gazebo/platoon processes"
            )

    # ---------------------------------------------------------
    # Report
    # ---------------------------------------------------------

    print()
    print("=" * 60)
    print("PRE-LAUNCH CHECKS")
    print("=" * 60)

    for message in passed:

        print(
            "[PASS]",
            message,
        )

    for message in failed:

        print(
            "[FAIL]",
            message,
        )

    print("=" * 60)

    if failed:

        raise RuntimeError(
            "Pre-launch checks failed."
        )

    print(
        "PRE-FLIGHT PASS: system is ready."
    )

    return True
