#!/usr/bin/env python3

"""
Select the existing autotuner for the requested tuning target.

This module builds commands only.
It does not start ROS, Gazebo, or Optuna.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


DEFAULT_REPO_ROOT = (
    Path.home()
    / "vrx_ws"
    / "src"
    / "vrx_platooning"
)


@dataclass
class TuningLaunchSpec:

    target: str
    executable: str
    arguments: List[str]

    def command(self) -> List[str]:

        return [
            "ros2",
            "run",
            "platoon_tuning",
            self.executable,
            *self.arguments,
        ]

    def command_string(self) -> str:

        return " ".join(
            self.command()
        )


def build_tuning_launch_spec(
    robot_count,
    tuning_target,
    *,
    run_name,
    follower_spacing=None,
    tuning_trials=None,
    tuning_time_limit_sec=None,
    repo_root=DEFAULT_REPO_ROOT,
):

    robot_count = int(robot_count)

    target = str(
        tuning_target
    ).strip().lower()

    if robot_count not in (
        1,
        2,
        3,
    ):
        raise ValueError(
            "robot_count must be 1, 2, or 3"
        )

    allowed = ["r1"]

    if robot_count >= 2:
        allowed.append("r2")

    if robot_count >= 3:
        allowed.append("r3")

    if target not in allowed:
        raise ValueError(
            f"Tuning target {target!r} is not "
            f"available for {robot_count} robot(s)."
        )

    repo_root = Path(
        repo_root
    ).expanduser()

    workspace = (
        repo_root
        .parent
        .parent
    )

    args = [
        "--study-name",
        str(run_name),

        "--workspace",
        str(workspace),
    ]

    if tuning_trials is not None:

        tuning_trials = int(
            tuning_trials
        )

        if tuning_trials <= 0:
            raise ValueError(
                "tuning_trials must be > 0"
            )

        args += [
            "--trials",
            str(tuning_trials),
        ]

    if tuning_time_limit_sec is not None:

        tuning_time_limit_sec = float(
            tuning_time_limit_sec
        )

        if tuning_time_limit_sec <= 0.0:
            raise ValueError(
                "tuning_time_limit_sec must be > 0"
            )

        args += [
            "--wall-timeout",
            str(tuning_time_limit_sec),
        ]

    # =========================================================
    # R1 tuning
    # =========================================================

    if target == "r1":

        controller_seed = (
            repo_root
            / "platoon_control"
            / "config"
            / "leader_pid.yaml"
        )

        planner_seed = (
            repo_root
            / "platoon_planner"
            / "config"
            / "planner.yaml"
        )

        for path in (
            controller_seed,
            planner_seed,
        ):
            if not path.exists():
                raise FileNotFoundError(path)

        args += [
            "--seed-controller",
            str(controller_seed),

            "--seed-planner",
            str(planner_seed),
        ]

        return TuningLaunchSpec(
            target="r1",
            executable="r1_stress_autotune_v3",
            arguments=args,
        )

    # Followers use the same V2.3 autotuner.
    if follower_spacing is None:
        follower_spacing = 5.0

    follower_spacing = float(
        follower_spacing
    )

    if follower_spacing <= 0.0:
        raise ValueError(
            "follower_spacing must be > 0"
        )

    args += [
        "--follower-spacing",
        str(follower_spacing),
    ]

    # =========================================================
    # R2 <- R1
    # =========================================================

    if target == "r2":

        controller_seed = (
            repo_root
            / "platoon_control"
            / "config"
            / "r2_follower_v22_tuned.yaml"
        )

        planner_seed = (
            repo_root
            / "platoon_planner"
            / "config"
            / "r2_follower_v22_tuned.yaml"
        )

        for path in (
            controller_seed,
            planner_seed,
        ):
            if not path.exists():
                raise FileNotFoundError(path)

        args += [
            "--launch-file",
            "follower_stress_pathgap_v22.launch.py",

            "--follower-id",
            "r2",

            "--predecessor-id",
            "r1",

            "--seed-controller",
            str(controller_seed),

            "--seed-planner",
            str(planner_seed),
        ]

        return TuningLaunchSpec(
            target="r2",
            executable="follower_stress_autotune_v23",
            arguments=args,
        )

    # =========================================================
    # R3 <- R2
    # =========================================================

    controller_seed = (
        repo_root
        / "platoon_control"
        / "config"
        / "r3_follower_v22_seed.yaml"
    )

    planner_seed = (
        repo_root
        / "platoon_planner"
        / "config"
        / "r3_follower_v22_seed.yaml"
    )

    for path in (
        controller_seed,
        planner_seed,
    ):
        if not path.exists():
            raise FileNotFoundError(path)

    args += [
        "--launch-file",
        "three_robot_r3_tuning.launch.py",

        "--follower-id",
        "r3",

        "--predecessor-id",
        "r2",

        "--seed-controller",
        str(controller_seed),

        "--seed-planner",
        str(planner_seed),
    ]

    return TuningLaunchSpec(
        target="r3",
        executable="follower_stress_autotune_v23",
        arguments=args,
    )


def build_tuning_launch_spec_from_config(
    config,
    *,
    repo_root=DEFAULT_REPO_ROOT,
):
    """
    Build the appropriate tuning command directly from StartupConfig.

    This function only builds a command specification.
    It does not launch anything.
    """

    if config.run_mode != "tuning":
        raise ValueError(
            "StartupConfig is not in tuning mode."
        )

    follower_spacing = (
        5.0
        if config.follower_spacing is None
        else float(config.follower_spacing)
    )

    return build_tuning_launch_spec(
        config.robot_count,
        config.tuning_target,
        run_name=config.run_name,
        follower_spacing=follower_spacing,
        tuning_trials=config.tuning_trials,
        tuning_time_limit_sec=(
            config.tuning_time_limit_sec
        ),
        repo_root=repo_root,
    )
