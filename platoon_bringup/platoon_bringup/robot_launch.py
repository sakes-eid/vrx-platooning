#!/usr/bin/env python3

"""
Robot-count launch selection.

This module decides which existing proven bringup should be used
for 1, 2, or 3 WAM-Vs.

Important:
    The built-in R1 Stress planner is always disabled here.

The approved mission planner will be started separately by the
top-level startup system.

Follower planners/controllers remain owned by their existing
multi-robot bringup files.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List


DEFAULT_REPO_ROOT = (
    Path.home()
    / "vrx_ws"
    / "src"
    / "vrx_platooning"
)


@dataclass
class RobotLaunchSpec:

    robot_count: int

    launch_file: str

    arguments: Dict[
        str,
        str,
    ]

    def command(
        self,
    ) -> List[str]:

        command = [
            "ros2",
            "launch",
            "platoon_bringup",
            self.launch_file,
        ]

        for key, value in (
            self.arguments.items()
        ):

            command.append(
                f"{key}:={value}"
            )

        return command

    def command_string(
        self,
    ) -> str:

        return " ".join(
            self.command()
        )


def _bool_text(
    value,
):

    return (
        "true"
        if value
        else "false"
    )


def build_robot_launch_spec(
    robot_count,
    *,
    headless=False,
    show_map=True,
    run_name="platoon_run",
    simulation_profile="light",
    follower_spacing=5.0,
    results_dir=None,
    repo_root=DEFAULT_REPO_ROOT,
):
    """
    Select the proven bringup stack for the requested robot count.

    This step intentionally uses only arguments currently supported
    by each launch file.

    Full argument standardization is handled in Step 6.
    """

    robot_count = int(
        robot_count
    )

    follower_spacing = float(
        follower_spacing
    )

    if follower_spacing <= 0.0:

        raise ValueError(
            "follower_spacing must be > 0."
        )

    repo_root = Path(
        repo_root
    ).expanduser()

    if results_dir is None:

        results_dir = (
            repo_root
            / "results"
        )

    results_dir = Path(
        results_dir
    ).expanduser()

    # ---------------------------------------------------------
    # R1 only
    # ---------------------------------------------------------

    if robot_count == 1:

        return RobotLaunchSpec(
            robot_count=1,

            launch_file=(
                "leader_stress_full.launch.py"
            ),

            arguments={
                "enable_planner":
                    "false",

                "run_name":
                    str(
                        run_name
                    ),

                "headless":
                    _bool_text(
                        headless
                    ),

                "show_map":
                    _bool_text(
                        show_map
                    ),

                "simulation_profile":
                    str(
                        simulation_profile
                    ),

                "results_dir":
                    str(
                        results_dir
                    ),
            },
        )

    # ---------------------------------------------------------
    # R1 + R2
    # ---------------------------------------------------------

    if robot_count == 2:

        controller_params = (
            repo_root
            / "platoon_control"
            / "config"
            / "r2_follower_v22_tuned.yaml"
        )

        planner_params = (
            repo_root
            / "platoon_planner"
            / "config"
            / "r2_follower_v22_tuned.yaml"
        )

        if not controller_params.exists():

            raise FileNotFoundError(
                "R2 controller parameters "
                f"not found: {controller_params}"
            )

        if not planner_params.exists():

            raise FileNotFoundError(
                "R2 planner parameters "
                f"not found: {planner_params}"
            )

        return RobotLaunchSpec(
            robot_count=2,

            launch_file=(
                "follower_stress_visual.launch.py"
            ),

            arguments={
                "controller_params_file":
                    str(
                        controller_params
                    ),

                "planner_params_file":
                    str(
                        planner_params
                    ),

                "enable_planner":
                    "false",

                "follower_spacing":
                    str(
                        follower_spacing
                    ),

                "run_name":
                    str(
                        run_name
                    ),

                "results_root":
                    str(
                        results_dir
                    ),

                "headless":
                    _bool_text(
                        headless
                    ),

                "show_map":
                    _bool_text(
                        show_map
                    ),
            },
        )

    # ---------------------------------------------------------
    # R1 + R2 + R3
    # ---------------------------------------------------------

    if robot_count == 3:

        return RobotLaunchSpec(
            robot_count=3,

            launch_file=(
                "three_robot_stress_visual.launch.py"
            ),

            arguments={
                "enable_planner":
                    "false",

                "follower_spacing":
                    str(
                        follower_spacing
                    ),

                "headless":
                    _bool_text(
                        headless
                    ),
            },
        )

    raise ValueError(
        "robot_count must be 1, 2, or 3."
    )
