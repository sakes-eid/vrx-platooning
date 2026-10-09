#!/usr/bin/env python3

"""
Save persistent VRX mission definitions.

Saved missions contain only reusable mission geometry.

They do NOT contain:
    - current R1 position
    - A* route
    - Dubins approach
    - robot count
    - controller configuration
    - run mode
"""

import json
import re

from pathlib import Path

from platoon_planner.mission_definition import (
    MissionDefinition,
    STANDARD_STRESS_COURSE,
)


DEFAULT_MISSION_DIRECTORY = (
    Path.home()
    / "vrx_ws"
    / "src"
    / "vrx_platooning"
    / "saved_missions"
)


def safe_filename(
    name,
):
    """
    Convert mission name into a safe JSON filename.
    """

    name = (
        name.strip()
        .lower()
    )

    name = re.sub(
        r"[^a-z0-9_-]+",
        "_",
        name,
    )

    name = name.strip(
        "_"
    )

    if not name:

        raise ValueError(
            "Mission name does not contain "
            "a valid filename."
        )

    return (
        name
        + ".json"
    )


def definition_from_plan(
    mission,
    name,
):
    """
    Convert an accepted MissionPlan into its reusable
    MissionDefinition.

    Only mission-defining geometry is retained.
    """

    mission_type = (
        mission.mission_type
        .strip()
        .lower()
    )

    parameters = (
        mission.parameters
    )

    if mission_type == "straight":

        definition = MissionDefinition(
            name=name,
            mission_type="straight",
            geometry={
                "start":
                    parameters[
                        "straight_start"
                    ],

                "end":
                    parameters[
                        "straight_end"
                    ],
            },
        )

    elif mission_type == "curve":

        definition = MissionDefinition(
            name=name,
            mission_type="curve",
            geometry={
                "start":
                    parameters[
                        "curve_start"
                    ],

                "apogee":
                    parameters[
                        "curve_apogee"
                    ],

                "end":
                    parameters[
                        "curve_end"
                    ],
            },
        )

    elif mission_type == "coverage":

        definition = MissionDefinition(
            name=name,
            mission_type="coverage",
            geometry={
                "first_corner":
                    parameters[
                        "first_corner"
                    ],

                "opposite_corner":
                    parameters[
                        "opposite_corner"
                    ],

                "line_spacing":
                    parameters[
                        "line_spacing"
                    ],
            },
        )

    elif mission_type == "stress":

        definition = MissionDefinition(
            name=name,
            mission_type="stress",
            geometry={
                "standard_course":
                    STANDARD_STRESS_COURSE,
            },
        )

    else:

        raise ValueError(
            f"Cannot save unsupported "
            f"mission type: {mission_type}"
        )

    definition.validate()

    return definition


def save_mission_definition(
    definition,
    directory=DEFAULT_MISSION_DIRECTORY,
    overwrite=False,
):
    """
    Save MissionDefinition as human-readable JSON.

    Existing files are protected unless overwrite=True.
    """

    definition.validate()

    directory = Path(
        directory
    )

    directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    filepath = (
        directory
        / safe_filename(
            definition.name
        )
    )

    if (
        filepath.exists()
        and not overwrite
    ):

        raise FileExistsError(
            f"Mission already exists: "
            f"{filepath.name}"
        )

    data = (
        definition.to_dict()
    )

    temporary = filepath.with_suffix(
        ".json.tmp"
    )

    with temporary.open(
        "w",
        encoding="utf-8",
    ) as handle:

        json.dump(
            data,
            handle,
            indent=2,
            sort_keys=False,
        )

        handle.write(
            "\n"
        )

    # Atomic replacement prevents half-written JSON files.
    temporary.replace(
        filepath
    )

    return filepath


def save_mission_plan(
    mission,
    name,
    directory=DEFAULT_MISSION_DIRECTORY,
    overwrite=False,
):
    """
    Convenience function:

        accepted MissionPlan
             ->
        MissionDefinition
             ->
        JSON file
    """

    definition = (
        definition_from_plan(
            mission,
            name,
        )
    )

    return save_mission_definition(
        definition,
        directory=directory,
        overwrite=overwrite,
    )


def load_mission_definition(
    filepath,
):
    """
    Load and validate one saved mission JSON file.
    """

    filepath = Path(
        filepath
    )

    if not filepath.exists():

        raise FileNotFoundError(
            f"Mission file not found: "
            f"{filepath}"
        )

    if filepath.suffix.lower() != ".json":

        raise ValueError(
            "Mission file must be JSON."
        )

    try:

        with filepath.open(
            "r",
            encoding="utf-8",
        ) as handle:

            data = json.load(
                handle
            )

    except json.JSONDecodeError as exc:

        raise ValueError(
            f"Invalid JSON in mission file: "
            f"{filepath.name}"
        ) from exc

    definition = (
        MissionDefinition.from_dict(
            data
        )
    )

    return definition


def list_saved_missions(
    directory=DEFAULT_MISSION_DIRECTORY,
):
    """
    Return validated saved missions.

    Each result contains:

        {
            "path": Path,
            "name": str,
            "mission_type": str,
            "definition": MissionDefinition,
        }

    Invalid files are skipped and reported separately.
    """

    directory = Path(
        directory
    )

    if not directory.exists():

        return (
            [],
            [],
        )

    missions = []
    invalid = []

    for filepath in sorted(
        directory.glob(
            "*.json"
        )
    ):

        try:

            definition = (
                load_mission_definition(
                    filepath
                )
            )

            missions.append(
                {
                    "path":
                        filepath,

                    "name":
                        definition.name,

                    "mission_type":
                        definition.mission_type,

                    "definition":
                        definition,
                }
            )

        except Exception as exc:

            invalid.append(
                {
                    "path":
                        filepath,

                    "error":
                        str(
                            exc
                        ),
                }
            )

    return (
        missions,
        invalid,
    )


def choose_saved_mission(
    directory=DEFAULT_MISSION_DIRECTORY,
):
    """
    Terminal selector for saved missions.

    Returns:
        MissionDefinition
        or None if cancelled.
    """

    missions, invalid = (
        list_saved_missions(
            directory
        )
    )

    if invalid:

        print()
        print(
            "Invalid saved mission files:"
        )

        for item in invalid:

            print(
                "  -",
                item[
                    "path"
                ].name,
                ":",
                item[
                    "error"
                ],
            )

    if not missions:

        print()
        print(
            "No valid saved missions found."
        )

        return None

    print()
    print(
        "Saved missions:"
    )

    for index, item in enumerate(
        missions,
        start=1,
    ):

        print(
            f"  {index}. "
            f"{item['name']} "
            f"[{item['mission_type']}]"
        )

    print(
        f"  {len(missions) + 1}. Cancel"
    )

    while True:

        answer = input(
            "Select mission: "
        ).strip()

        try:

            choice = int(
                answer
            )

        except ValueError:

            print(
                "Enter a mission number."
            )

            continue

        if choice == (
            len(
                missions
            )
            + 1
        ):

            return None

        if (
            1
            <= choice
            <= len(
                missions
            )
        ):

            return missions[
                choice - 1
            ][
                "definition"
            ]

        print(
            "Invalid selection."
        )
