#!/usr/bin/env python3

"""
Top-level VRX platooning startup interface.

STEP 1:
    Collect and validate configuration only.

Nothing is launched yet.
"""

from platoon_bringup.startup_config import (
    StartupConfig,
)

from platoon_planner.mission_storage import (
    choose_saved_mission,
    DEFAULT_MISSION_DIRECTORY,
    safe_filename,
)


def ask_choice(
    title,
    choices,
    default,
):

    print()
    print(title)

    for number, label, _ in choices:

        print(
            f"  {number}. {label}"
        )

    while True:

        answer = input(
            f"Select [{default}]: "
        ).strip()

        if answer == "":
            answer = str(
                default
            )

        for number, _, value in choices:

            if answer == str(
                number
            ):
                return value

        print(
            "Invalid selection."
        )


def ask_yes_no(
    question,
    default,
):

    prompt = (
        "[Y/n]"
        if default
        else "[y/N]"
    )

    while True:

        answer = input(
            f"{question} {prompt}: "
        ).strip().lower()

        if answer == "":

            return default

        if answer in (
            "y",
            "yes",
        ):

            return True

        if answer in (
            "n",
            "no",
        ):

            return False

        print(
            "Enter y or n."
        )


def ask_positive_float(
    question,
    default,
):

    while True:

        answer = input(
            f"{question} [{default}]: "
        ).strip()

        if answer == "":

            return float(
                default
            )

        try:

            value = float(
                answer
            )

            if value <= 0.0:
                raise ValueError

            return value

        except ValueError:

            print(
                "Enter a positive number."
            )


def ask_optional_positive_float(
    question,
):

    while True:

        answer = input(
            f"{question} "
            "[blank = use system default]: "
        ).strip()

        if answer == "":

            return None

        try:

            value = float(
                answer
            )

            if value <= 0.0:
                raise ValueError

            return value

        except ValueError:

            print(
                "Enter a positive number "
                "or leave blank."
            )


def ask_optional_positive_int(
    question,
):

    while True:

        answer = input(
            f"{question} "
            "[blank = use tuner default]: "
        ).strip()

        if answer == "":

            return None

        try:

            value = int(
                answer
            )

            if value <= 0:
                raise ValueError

            return value

        except ValueError:

            print(
                "Enter a positive integer "
                "or leave blank."
            )


def tuning_target_label(
    target,
):

    labels = {
        "r1":
            "R1",

        "r2":
            "R2 <- R1",

        "r3":
            "R3 <- R2",
    }

    return labels[
        target
    ]


def collect_configuration():

    config = StartupConfig()

    print()
    print("=" * 60)
    print("VRX PLATOONING STARTUP")
    print("=" * 60)

    # ---------------------------------------------------------
    # Robots
    # ---------------------------------------------------------

    config.robot_count = int(
        ask_choice(
            "Number of robots:",
            (
                (
                    1,
                    "R1",
                    1,
                ),
                (
                    2,
                    "R1 + R2",
                    2,
                ),
                (
                    3,
                    "R1 + R2 + R3",
                    3,
                ),
            ),
            default=1,
        )
    )

    # ---------------------------------------------------------
    # Mission source + mission type
    # ---------------------------------------------------------

    while True:

        config.mission_source = ask_choice(
            "Mission source:",
            (
                (
                    1,
                    "Create new mission",
                    "new",
                ),
                (
                    2,
                    "Load saved mission",
                    "saved",
                ),
            ),
            default=1,
        )

        # =====================================================
        # New mission
        # =====================================================

        if config.mission_source == "new":

            config.saved_mission_name = None
            config.saved_mission_file = None

            config.mission_type = ask_choice(
                "Mission / path type:",
                (
                    (
                        1,
                        "Straight",
                        "straight",
                    ),
                    (
                        2,
                        "Curve",
                        "curve",
                    ),
                    (
                        3,
                        "Coverage",
                        "coverage",
                    ),
                    (
                        4,
                        "Standard stress course",
                        "stress",
                    ),
                ),
                default=4,
            )

            config.save_after_accept = (
                ask_yes_no(
                    "Save this mission after "
                    "it is validated and accepted?",
                    default=False,
                )
            )

            if config.save_after_accept:

                while True:

                    name = input(
                        "Saved mission name: "
                    ).strip()

                    if name:

                        config.save_mission_name = (
                            name
                        )

                        break

                    print(
                        "Mission name must not "
                        "be empty."
                    )

            else:

                config.save_mission_name = None

            break

        # =====================================================
        # Load saved mission
        # =====================================================

        definition = (
            choose_saved_mission()
        )

        if definition is None:

            print()
            print(
                "Saved mission selection cancelled."
            )

            print(
                "Returning to mission source..."
            )

            continue

        config.saved_mission_name = (
            definition.name
        )

        config.saved_mission_file = str(
            DEFAULT_MISSION_DIRECTORY
            / safe_filename(
                definition.name
            )
        )

        config.mission_type = (
            definition.mission_type
        )

        # Keep a copy of the persistent definition available
        # to the future master launcher.
        config.mission_parameters = (
            definition.to_dict()
        )

        config.save_after_accept = False
        config.save_mission_name = None

        print()
        print(
            "Loaded saved mission:"
        )

        print(
            "  Name :",
            definition.name,
        )

        print(
            "  Type :",
            definition.mission_type,
        )

        break

    # ---------------------------------------------------------
    # Test / tuning
    # ---------------------------------------------------------

    config.run_mode = ask_choice(
        "Run mode:",
        (
            (
                1,
                "Test run",
                "test",
            ),
            (
                2,
                "Tuning run",
                "tuning",
            ),
        ),
        default=1,
    )

    # ---------------------------------------------------------
    # Simulation profile
    # ---------------------------------------------------------

    config.simulation_profile = ask_choice(
        "Simulation profile:",
        (
            (
                1,
                "Light",
                "light",
            ),
            (
                2,
                "Full",
                "full",
            ),
        ),
        default=1,
    )

    # ---------------------------------------------------------
    # Display
    # ---------------------------------------------------------

    config.headless = ask_yes_no(
        "Run Gazebo headless?",
        default=False,
    )

    config.show_rviz = ask_yes_no(
        "Launch RViz?",
        default=True,
    )

    if config.show_rviz:

        config.show_map = ask_yes_no(
            "Show Sydney shoreline/map?",
            default=True,
        )

    else:

        config.show_map = False

    # ---------------------------------------------------------
    # Mission speed
    # ---------------------------------------------------------

    config.max_speed = (
        ask_positive_float(
            "Maximum mission speed [m/s]",
            default=2.0,
        )
    )

    # ---------------------------------------------------------
    # Followers
    # ---------------------------------------------------------

    if config.robot_count > 1:

        config.follower_spacing = (
            ask_optional_positive_float(
                "Follower spacing [m]"
            )
        )

    # ---------------------------------------------------------
    # Tuning
    # ---------------------------------------------------------

    if config.run_mode == "tuning":

        allowed = (
            config.allowed_tuning_targets()
        )

        choices = []

        for index, target in enumerate(
            allowed,
            start=1,
        ):

            choices.append(
                (
                    index,
                    tuning_target_label(
                        target
                    ),
                    target,
                )
            )

        config.tuning_target = (
            ask_choice(
                "Tuning target:",
                tuple(
                    choices
                ),
                default=1,
            )
        )

        config.tuning_trials = (
            ask_optional_positive_int(
                "Number of tuning trials"
            )
        )

        config.tuning_time_limit_sec = (
            ask_optional_positive_float(
                "Tuning time limit [s]"
            )
        )

    # ---------------------------------------------------------
    # Logging
    # ---------------------------------------------------------

    config.logging_enabled = (
        ask_yes_no(
            "Enable result logging?",
            default=True,
        )
    )

    if config.logging_enabled:

        answer = input(
            "Results directory "
            f"[{config.results_dir}]: "
        ).strip()

        if answer:

            config.results_dir = (
                answer
            )

    # ---------------------------------------------------------
    # Run name
    # ---------------------------------------------------------

    temporary_name = (
        config.default_run_name()
    )

    answer = input(
        "Run name "
        f"[{temporary_name}]: "
    ).strip()

    config.run_name = (
        answer
        if answer
        else temporary_name
    )

    config.finalize()

    return config


def print_summary(
    config,
):

    follower_spacing = (
        "system default"
        if config.follower_spacing is None
        else f"{config.follower_spacing:.2f} m"
    )

    print()
    print("=" * 60)
    print("STARTUP CONFIGURATION")
    print("=" * 60)

    print(
        "Robot chain        :",
        config.robot_chain(),
    )

    print(
        "Mission            :",
        config.mission_type,
    )

    print(
        "Mission source     :",
        config.mission_source,
    )

    if config.mission_source == "saved":

        print(
            "Saved mission     :",
            config.saved_mission_name,
        )

    if (
        config.mission_source == "new"
        and config.save_after_accept
    ):

        print(
            "Save after accept :",
            True,
        )

        print(
            "Save mission as   :",
            config.save_mission_name,
        )

    print(
        "Mission geometry   :",
        "not configured yet (Step 2)",
    )

    print(
        "Run mode           :",
        config.run_mode,
    )

    print(
        "Run name           :",
        config.run_name,
    )

    print(
        "Simulation profile :",
        config.simulation_profile,
    )

    print(
        "Gazebo headless    :",
        config.headless,
    )

    print(
        "RViz               :",
        config.show_rviz,
    )

    print(
        "Sydney map         :",
        config.show_map,
    )

    print(
        "Maximum speed      :",
        f"{config.max_speed:.2f} m/s",
    )

    if config.robot_count > 1:

        print(
            "Follower spacing   :",
            follower_spacing,
        )

    if config.run_mode == "tuning":

        print(
            "Tuning target      :",
            tuning_target_label(
                config.tuning_target
            ),
        )

        print(
            "Tuning trials      :",
            (
                config.tuning_trials
                if config.tuning_trials
                is not None
                else "tuner default"
            ),
        )

        print(
            "Tuning time limit  :",
            (
                f"{config.tuning_time_limit_sec:.0f} s"
                if
                config.tuning_time_limit_sec
                is not None
                else "tuner default"
            ),
        )

    print(
        "Logging            :",
        config.logging_enabled,
    )

    if config.logging_enabled:

        print(
            "Results directory  :",
            config.results_dir,
        )

    print("=" * 60)

    print()
    print(
        "STEP 1 COMPLETE:"
    )

    print(
        "Configuration validated."
    )

    print(
        "Nothing has been launched."
    )


def main():

    config = (
        collect_configuration()
    )

    print_summary(
        config
    )


if __name__ == "__main__":
    main()
