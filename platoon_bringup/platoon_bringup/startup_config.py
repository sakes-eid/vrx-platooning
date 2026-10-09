#!/usr/bin/env python3

"""
Shared startup configuration for the VRX platooning system.

This module contains configuration only.
It does not launch ROS, Gazebo, RViz, planners, or controllers.
"""

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional


VALID_MISSIONS = (
    "straight",
    "curve",
    "coverage",
    "stress",
)

VALID_RUN_MODES = (
    "test",
    "tuning",
)

VALID_SIMULATION_PROFILES = (
    "light",
)


@dataclass
class StartupConfig:

    # ---------------------------------------------------------
    # Platoon
    # ---------------------------------------------------------

    robot_count: int = 1

    # R1 is always leader.
    # R2 follows R1.
    # R3 follows R2.
    follower_spacing: Optional[float] = None

    # ---------------------------------------------------------
    # Mission
    # ---------------------------------------------------------

    mission_type: str = "stress"

    # Mission source:
    #   new   -> user defines a new mission
    #   saved -> mission geometry comes from saved_missions/
    mission_source: str = "new"

    saved_mission_name: Optional[str] = None
    saved_mission_file: Optional[str] = None

    # For newly created missions, the actual save occurs only
    # after the mission has passed validation and been accepted.
    save_after_accept: bool = False
    save_mission_name: Optional[str] = None

    # Mission-specific geometry will be populated later.
    mission_parameters: Dict[str, Any] = field(
        default_factory=dict
    )

    mission_name: Optional[str] = None

    # Planner maximum requested speed.
    # Curvature logic may command less.
    max_speed: float = 2.0

    # ---------------------------------------------------------
    # Run
    # ---------------------------------------------------------

    run_mode: str = "test"
    run_name: str = ""

    # ---------------------------------------------------------
    # Simulation / visualization
    # ---------------------------------------------------------

    simulation_profile: str = "light"

    headless: bool = False

    show_rviz: bool = True

    # Sydney shoreline / mission map in RViz.
    show_map: bool = True

    # ---------------------------------------------------------
    # Logging
    # ---------------------------------------------------------

    logging_enabled: bool = True

    results_dir: str = str(
        Path.home()
        / "vrx_ws"
        / "src"
        / "vrx_platooning"
        / "results"
    )

    # ---------------------------------------------------------
    # Tuning
    # ---------------------------------------------------------

    # r1
    # r2  = R2 <- R1
    # r3  = R3 <- R2
    tuning_target: Optional[str] = None

    tuning_trials: Optional[int] = None

    tuning_time_limit_sec: Optional[float] = None

    # ---------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------

    def robot_chain(self):

        if self.robot_count == 1:
            return "R1"

        if self.robot_count == 2:
            return "R1 -> R2"

        return "R1 -> R2 -> R3"

    def allowed_tuning_targets(self):

        targets = [
            "r1",
        ]

        if self.robot_count >= 2:
            targets.append(
                "r2"
            )

        if self.robot_count >= 3:
            targets.append(
                "r3"
            )

        return targets

    def default_run_name(self):

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        return (
            f"r{self.robot_count}_"
            f"{self.mission_type}_"
            f"{self.run_mode}_"
            f"{timestamp}"
        )

    def finalize(self):

        self.mission_type = (
            self.mission_type
            .strip()
            .lower()
        )

        self.run_mode = (
            self.run_mode
            .strip()
            .lower()
        )

        self.simulation_profile = (
            self.simulation_profile
            .strip()
            .lower()
        )

        if self.tuning_target is not None:

            self.tuning_target = (
                self.tuning_target
                .strip()
                .lower()
            )

        if not self.run_name.strip():

            self.run_name = (
                self.default_run_name()
            )

        # Test runs do not use tuning parameters.
        if self.run_mode == "test":

            self.tuning_target = None
            self.tuning_trials = None
            self.tuning_time_limit_sec = None

        self.validate()

    def validate(self):

        if self.robot_count not in (
            1,
            2,
            3,
        ):

            raise ValueError(
                "robot_count must be 1, 2, or 3"
            )

        if self.mission_source not in (
            "new",
            "saved",
        ):

            raise ValueError(
                "mission_source must be "
                "new or saved"
            )

        if self.mission_source == "saved":

            if not self.saved_mission_name:

                raise ValueError(
                    "A saved mission must be selected."
                )

            if not self.saved_mission_file:

                raise ValueError(
                    "Saved mission file is missing."
                )

            # A loaded mission already exists, so do not
            # create another copy after acceptance.
            self.save_after_accept = False
            self.save_mission_name = None

        if (
            self.mission_source == "new"
            and self.save_after_accept
        ):

            if (
                self.save_mission_name is None
                or not self.save_mission_name.strip()
            ):

                raise ValueError(
                    "A mission save name is required."
                )

        if (
            self.mission_type
            not in VALID_MISSIONS
        ):

            raise ValueError(
                "Invalid mission_type"
            )

        if (
            self.run_mode
            not in VALID_RUN_MODES
        ):

            raise ValueError(
                "Invalid run_mode"
            )

        if (
            self.simulation_profile
            not in VALID_SIMULATION_PROFILES
        ):

            raise ValueError(
                "Invalid simulation_profile"
            )

        if self.max_speed <= 0.0:

            raise ValueError(
                "max_speed must be > 0"
            )

        if (
            self.follower_spacing is not None
            and
            self.follower_spacing <= 0.0
        ):

            raise ValueError(
                "follower_spacing must be > 0"
            )

        if (
            self.run_mode == "tuning"
        ):

            if (
                self.tuning_target
                not in self.allowed_tuning_targets()
            ):

                raise ValueError(
                    "Tuning target is incompatible "
                    "with selected robot count."
                )

            if (
                self.tuning_trials is not None
                and
                self.tuning_trials <= 0
            ):

                raise ValueError(
                    "tuning_trials must be > 0"
                )

            if (
                self.tuning_time_limit_sec is not None
                and
                self.tuning_time_limit_sec <= 0.0
            ):

                raise ValueError(
                    "tuning_time_limit_sec must be > 0"
                )
