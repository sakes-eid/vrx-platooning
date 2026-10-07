#!/usr/bin/env python3

"""
Common preview / approval layer for all mission types.

The preview is intentionally separate from mission generation.

Input:
    MissionPlan
    Sydney navigation map
    optional mission-specific important points

Output:
    "accept"
    "modify"
    "cancel"
"""

import matplotlib.pyplot as plt

from matplotlib.colors import (
    ListedColormap,
)

from platoon_planner.mission_point_selector import (
    display_map,
)


def print_mission_summary(
    mission,
):
    """
    Print a mission-independent summary.
    """

    print()
    print("=" * 60)
    print("MISSION PREVIEW")
    print("=" * 60)

    print(
        "Mission type       :",
        mission.mission_type,
    )

    print(
        "Waypoint count     :",
        mission.waypoint_count,
    )

    print(
        "Complete length    :",
        f"{mission.path_length:.2f} m",
    )

    if mission.minimum_clearance is None:

        clearance_text = (
            "not yet validated"
        )

    else:

        clearance_text = (
            f"{mission.minimum_clearance:.2f} m"
        )

    print(
        "Minimum clearance  :",
        clearance_text,
    )

    print(
        "Collision checked  :",
        mission.collision_checked,
    )

    if mission.collision_checked:

        print(
            "Collision result   : PASS"
        )

    else:

        print(
            "Collision result   : NOT CHECKED"
        )

    # --------------------------------------------------------
    # Mission-specific useful diagnostics
    # --------------------------------------------------------

    diagnostics = (
        mission.diagnostics
    )

    if "straight_length" in diagnostics:

        print(
            "Straight length    :",
            f"{diagnostics['straight_length']:.2f} m",
        )

    if "curve_length" in diagnostics:

        print(
            "Curve length       :",
            f"{diagnostics['curve_length']:.2f} m",
        )

    if "coverage_segments" in diagnostics:

        print(
            "Coverage segments  :",
            diagnostics[
                "coverage_segments"
            ],
        )

    if "astar_waypoints" in diagnostics:

        print(
            "A* waypoints       :",
            diagnostics[
                "astar_waypoints"
            ],
        )

    if "approach_astar_waypoints" in diagnostics:

        print(
            "A* waypoints       :",
            diagnostics[
                "approach_astar_waypoints"
            ],
        )

    if "spawn_escape_used" in diagnostics:

        print(
            "Spawn escape       :",
            diagnostics[
                "spawn_escape_used"
            ],
        )

    print("=" * 60)


def plot_mission_preview(
    mission,
    raw_grid,
    safe_grid,
    metadata,
    r1_start_ned=None,
    important_points=None,
):
    """
    Plot complete mission on the Sydney navigation map.

    important_points:
        optional list of dictionaries:

        {
            "label": "Curve Start",
            "point": (north, east),
            "marker": "x",
        }
    """

    display = display_map(
        raw_grid,
        safe_grid,
    )

    cmap = ListedColormap(
        [
            "white",
            "orange",
            "black",
        ]
    )

    fig, ax = plt.subplots(
        figsize=(
            15,
            9,
        )
    )

    ax.imshow(
        display,
        origin="lower",
        extent=[
            metadata[
                "east_min"
            ],
            metadata[
                "east_max"
            ],
            metadata[
                "north_min"
            ],
            metadata[
                "north_max"
            ],
        ],
        interpolation="nearest",
        aspect="equal",
        cmap=cmap,
        vmin=0,
        vmax=2,
    )

    path = (
        mission.path_points
    )

    ax.plot(
        [
            point[1]
            for point in path
        ],
        [
            point[0]
            for point in path
        ],
        linewidth=2.5,
        label="Complete R1 mission",
    )

    # --------------------------------------------------------
    # Show R1 start
    # --------------------------------------------------------

    if r1_start_ned is not None:

        ax.scatter(
            [
                r1_start_ned[1]
            ],
            [
                r1_start_ned[0]
            ],
            s=110,
            marker="o",
            label="R1 start",
        )

    # --------------------------------------------------------
    # Show important mission points
    # --------------------------------------------------------

    if important_points:

        for item in important_points:

            point = item[
                "point"
            ]

            ax.scatter(
                [
                    point[1]
                ],
                [
                    point[0]
                ],
                s=120,
                marker=item.get(
                    "marker",
                    "x",
                ),
                label=item[
                    "label"
                ],
            )

    ax.set_xlabel(
        "East [m]"
    )

    ax.set_ylabel(
        "North [m]"
    )

    title = (
        mission.mission_type
        .replace(
            "_",
            " ",
        )
        .title()
    )

    ax.set_title(
        f"{title} Mission Preview"
    )

    ax.legend()

    plt.show()


def ask_mission_decision(
    allow_modify=True,
):
    """
    Return:
        accept
        modify
        cancel
    """

    print()
    print(
        "Mission decision:"
    )

    print(
        "  1. Accept mission"
    )

    if allow_modify:

        print(
            "  2. Modify mission"
        )

        print(
            "  3. Cancel"
        )

    else:

        print(
            "  2. Cancel"
        )

    while True:

        answer = input(
            "Select [1]: "
        ).strip()

        if answer == "":
            answer = "1"

        if answer == "1":

            return "accept"

        if (
            allow_modify
            and answer == "2"
        ):

            return "modify"

        if (
            allow_modify
            and answer == "3"
        ):

            return "cancel"

        if (
            not allow_modify
            and answer == "2"
        ):

            return "cancel"

        if allow_modify:

            print(
                "Choose 1, 2, or 3."
            )

        else:

            print(
                "Choose 1 or 2."
            )


def preview_and_approve(
    mission,
    raw_grid,
    safe_grid,
    metadata,
    r1_start_ned=None,
    important_points=None,
    allow_modify=True,
):
    """
    Common preview entry point.
    """

    mission.validate()

    print_mission_summary(
        mission
    )

    plot_mission_preview(
        mission=mission,
        raw_grid=raw_grid,
        safe_grid=safe_grid,
        metadata=metadata,
        r1_start_ned=r1_start_ned,
        important_points=important_points,
    )

    return ask_mission_decision(
        allow_modify=allow_modify,
    )
