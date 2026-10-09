#!/usr/bin/env python3

"""
Build a fresh executable MissionPlan from a saved MissionDefinition.

Important:
    Saved mission geometry is reused.

    The R1 approach is NOT reused.

    A fresh:
        spawn escape
        A*
        Dubins approach

    is generated from the current live R1 position.
"""

from platoon_planner.straight_mission import (
    build_straight_mission_plan,
)

from platoon_planner.curve_mission import (
    build_curve_mission_plan,
)

from platoon_planner.coverage_router import (
    build_coverage_mission_plan,
)

from platoon_planner.stress_mission import (
    build_standard_stress_mission_plan,
)

from platoon_planner.mission_validation import (
    validate_complete_mission,
)


def build_plan_from_definition(
    definition,
    r1_start_ned,
    r1_start_heading,
    raw_grid,
    safe_grid,
    clearance,
    metadata,
):
    """
    Convert a validated MissionDefinition into a fresh MissionPlan.
    """

    definition.validate()

    mission_type = (
        definition.mission_type
    )

    geometry = (
        definition.geometry
    )

    # ---------------------------------------------------------
    # Straight
    # ---------------------------------------------------------

    if mission_type == "straight":

        return build_straight_mission_plan(
            selected_start=(
                geometry[
                    "start"
                ]
            ),
            selected_end=(
                geometry[
                    "end"
                ]
            ),
            r1_start_ned=(
                r1_start_ned
            ),
            r1_start_heading=(
                r1_start_heading
            ),
            raw_grid=raw_grid,
            safe_grid=safe_grid,
            clearance=clearance,
            metadata=metadata,
        )

    # ---------------------------------------------------------
    # Curve
    # ---------------------------------------------------------

    if mission_type == "curve":

        return build_curve_mission_plan(
            selected_start=(
                geometry[
                    "start"
                ]
            ),
            selected_apogee=(
                geometry[
                    "apogee"
                ]
            ),
            selected_end=(
                geometry[
                    "end"
                ]
            ),
            r1_start_ned=(
                r1_start_ned
            ),
            r1_start_heading=(
                r1_start_heading
            ),
            raw_grid=raw_grid,
            safe_grid=safe_grid,
            clearance=clearance,
            metadata=metadata,
        )

    # ---------------------------------------------------------
    # Coverage
    # ---------------------------------------------------------

    if mission_type == "coverage":

        return build_coverage_mission_plan(
            first=(
                geometry[
                    "first_corner"
                ]
            ),
            second=(
                geometry[
                    "opposite_corner"
                ]
            ),
            line_spacing=(
                geometry[
                    "line_spacing"
                ]
            ),
            r1_start_ned=(
                r1_start_ned
            ),
            r1_start_heading=(
                r1_start_heading
            ),
            raw_grid=raw_grid,
            safe_grid=safe_grid,
            clearance=clearance,
            metadata=metadata,
        )

    # ---------------------------------------------------------
    # Stress
    # ---------------------------------------------------------

    if mission_type == "stress":

        mission = (
            build_standard_stress_mission_plan(
                start_ned=r1_start_ned,
            )
        )

        validation = (
            validate_complete_mission(
                mission.path_points,
                raw_grid,
                safe_grid,
                clearance,
                metadata,
            )
        )

        if not validation[
            "safe"
        ]:

            raise RuntimeError(
                "Loaded Standard Stress mission "
                "failed collision validation: "
                f"{validation}"
            )

        mission.minimum_clearance = float(
            validation[
                "minimum_clearance"
            ]
        )

        mission.collision_checked = True

        mission.diagnostics.update(
            {
                "spawn_escape_used":
                    bool(
                        validation[
                            "spawn_escape_used"
                        ]
                    ),

                "safe_start_index":
                    int(
                        validation[
                            "safe_start_index"
                        ]
                    ),
            }
        )

        return mission

    raise ValueError(
        f"Unsupported mission type: "
        f"{mission_type}"
    )
