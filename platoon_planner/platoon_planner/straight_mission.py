#!/usr/bin/env python3

"""
Map-aware straight mission generator.

Mission structure:

    live R1
        -> A*
        -> Dubins
        -> selected straight START
        -> exact straight segment
        -> selected END

The selected START -> END section itself must be completely safe.
If terrain or the WAM-V safety buffer intersects that segment,
the straight mission is rejected.
"""

import math

import numpy as np

from platoon_planner.mission_plan import (
    MissionPlan,
)

from platoon_planner.dubins_router import (
    heading_between,
    validate_path_on_safe_grid,
)

from platoon_planner.mission_approach import (
    build_mission_approach,
)


DEFAULT_APPROACH_TURN_RADIUS = 10.0
DEFAULT_SAMPLE_STEP = 0.5


def sample_straight(
    start,
    end,
    step=DEFAULT_SAMPLE_STEP,
):
    """
    Densely sample an exact straight line in world_ned.
    """

    start_north = float(
        start[0]
    )

    start_east = float(
        start[1]
    )

    end_north = float(
        end[0]
    )

    end_east = float(
        end[1]
    )

    dn = (
        end_north
        - start_north
    )

    de = (
        end_east
        - start_east
    )

    length = math.hypot(
        dn,
        de,
    )

    if length <= 1e-6:
        raise ValueError(
            "Straight start and end must "
            "be different points."
        )

    count = max(
        1,
        int(
            math.ceil(
                length / float(step)
            )
        ),
    )

    ratios = np.linspace(
        0.0,
        1.0,
        count + 1,
    )

    path = np.column_stack(
        (
            start_north
            + ratios * dn,

            start_east
            + ratios * de,
        )
    )

    return path


def validate_straight_segment(
    start,
    end,
    safe_grid,
    clearance,
    metadata,
):
    """
    Validate the ENTIRE selected straight, not only its endpoints.
    """

    path = np.asarray(
        [
            (
                float(start[0]),
                float(start[1]),
            ),
            (
                float(end[0]),
                float(end[1]),
            ),
        ],
        dtype=float,
    )

    return validate_path_on_safe_grid(
        path,
        safe_grid,
        clearance,
        metadata,
    )


def build_straight_mission_plan(
    selected_start,
    selected_end,
    r1_start_ned,
    r1_start_heading,
    raw_grid,
    safe_grid,
    clearance,
    metadata,
    approach_turn_radius=DEFAULT_APPROACH_TURN_RADIUS,
):
    """
    Build the complete R1 straight mission.

    The user-selected straight itself is never A*-rerouted.
    If it intersects unsafe space, the mission is rejected.

    A* + Dubins are used only to bring R1 safely to the selected
    starting point while aligned with the straight.
    """

    selected_start = (
        float(
            selected_start[0]
        ),
        float(
            selected_start[1]
        ),
    )

    selected_end = (
        float(
            selected_end[0]
        ),
        float(
            selected_end[1]
        ),
    )

    # ========================================================
    # 1. Validate exact selected straight
    # ========================================================

    straight_validation = (
        validate_straight_segment(
            selected_start,
            selected_end,
            safe_grid,
            clearance,
            metadata,
        )
    )

    if not straight_validation[
        "safe"
    ]:

        raise RuntimeError(
            "Selected START -> END straight "
            "crosses terrain or the WAM-V "
            "safety buffer."
        )

    straight_heading = (
        heading_between(
            selected_start,
            selected_end,
        )
    )

    # ========================================================
    # 2. Describe straight START as an approach target
    #
    # build_approach_path() currently belongs to coverage,
    # but its routing engine only requires:
    #
    #   start_cell
    #   start_ned
    #   heading
    #
    # We reuse it here without altering its proven behaviour.
    # ========================================================

    (
        approach,
        approach_grid,
        approach_validation,
    ) = build_mission_approach(
        start_ned=r1_start_ned,
        start_heading=r1_start_heading,
        target_ned=selected_start,
        target_heading=straight_heading,
        turning_radius=approach_turn_radius,
        safe_grid=safe_grid,
        clearance=clearance,
        metadata=metadata,
        raw_grid=raw_grid,
    )

    # ========================================================
    # 3. Build exact straight run
    # ========================================================

    straight_path = sample_straight(
        selected_start,
        selected_end,
        step=DEFAULT_SAMPLE_STEP,
    )

    # Avoid duplicating START because approach already ends there.
    complete_path = np.vstack(
        (
            approach[
                :,
                :2
            ],

            straight_path[
                1:
            ],
        )
    )

    # ========================================================
    # 4. Validate complete mission after R1 has escaped any
    #    exceptional spawn buffer.
    # ========================================================

    safe_start_index = int(
        approach_validation.get(
            "safe_start_index",
            0,
        )
    )

    complete_validation = (
        validate_path_on_safe_grid(
            complete_path[
                safe_start_index:
            ],
            safe_grid,
            clearance,
            metadata,
        )
    )

    if not complete_validation[
        "safe"
    ]:

        raise RuntimeError(
            "Combined R1 approach + straight "
            "mission failed collision validation."
        )

    minimum_clearance = min(
        float(
            approach_validation[
                "minimum_clearance"
            ]
        ),
        float(
            complete_validation[
                "minimum_clearance"
            ]
        ),
    )

    # ========================================================
    # 5. Common MissionPlan
    # ========================================================

    mission = MissionPlan(
        mission_type="straight",

        path_points=[
            (
                float(point[0]),
                float(point[1]),
            )
            for point
            in complete_path
        ],

        parameters={
            "straight_start":
                selected_start,

            "straight_end":
                selected_end,

            "straight_heading":
                float(
                    straight_heading
                ),

            "approach_turn_radius":
                float(
                    approach_turn_radius
                ),
        },

        minimum_clearance=(
            minimum_clearance
        ),

        collision_checked=True,

        diagnostics={
            "straight_length":
                float(
                    math.hypot(
                        selected_end[0]
                        - selected_start[0],

                        selected_end[1]
                        - selected_start[1],
                    )
                ),

            "approach_samples":
                int(
                    len(
                        approach
                    )
                ),

            "straight_samples":
                int(
                    len(
                        straight_path
                    )
                ),

            "astar_waypoints":
                int(
                    len(
                        approach_grid
                    )
                ),

            "spawn_escape_used":
                bool(
                    approach_validation.get(
                        "escape_used",
                        False,
                    )
                ),

            "safe_start_index":
                safe_start_index,

            "straight_minimum_clearance":
                float(
                    straight_validation[
                        "minimum_clearance"
                    ]
                ),
        },
    )

    mission.validate()

    return mission
