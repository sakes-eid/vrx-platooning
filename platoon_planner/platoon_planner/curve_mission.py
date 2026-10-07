#!/usr/bin/env python3

"""
Map-aware three-point curve mission generator.

User selects:

    1. Curve Start
    2. Apogee
    3. Curve End

The generated quadratic curve passes exactly through all three
selected points.

Complete mission:

    live R1
        -> A*
        -> Dubins
        -> align with curve start tangent
        -> Curve Start
        -> Apogee
        -> Curve End

The curve itself is never A*-rerouted. If any part intersects
terrain or the WAM-V safety buffer, the mission is rejected.
"""

import math

import numpy as np

from platoon_planner.mission_plan import (
    MissionPlan,
)

from platoon_planner.mission_approach import (
    build_mission_approach,
)

from platoon_planner.dubins_router import (
    validate_path_on_safe_grid,
)


DEFAULT_APPROACH_TURN_RADIUS = 10.0
DEFAULT_SAMPLE_STEP = 0.5


def _point(
    value,
):
    return np.asarray(
        [
            float(value[0]),
            float(value[1]),
        ],
        dtype=float,
    )


def curve_control_point(
    start,
    apogee,
    end,
):
    """
    Return the quadratic Bezier control point required so that:

        P(0.0) = start
        P(0.5) = apogee
        P(1.0) = end

    For a quadratic Bezier:

        P(t) =
            (1-t)^2 P0
            + 2(1-t)t B
            + t^2 P2

    Solving at t=0.5 gives:

        B = 2*apogee - 0.5*(start + end)
    """

    start = _point(
        start
    )

    apogee = _point(
        apogee
    )

    end = _point(
        end
    )

    return (
        2.0 * apogee
        - 0.5 * (
            start
            + end
        )
    )


def evaluate_curve(
    start,
    control,
    end,
    t,
):
    """
    Evaluate quadratic curve at parameter t.
    """

    t = np.asarray(
        t,
        dtype=float,
    )

    one_minus_t = (
        1.0 - t
    )

    return (
        one_minus_t[
            ...,
            None
        ] ** 2
        * start

        + 2.0
        * one_minus_t[
            ...,
            None
        ]
        * t[
            ...,
            None
        ]
        * control

        + t[
            ...,
            None
        ] ** 2
        * end
    )


def curve_tangent(
    start,
    control,
    end,
    t,
):
    """
    Tangent vector of the quadratic curve.
    """

    return (
        2.0
        * (
            (1.0 - t)
            * (
                control - start
            )

            + t
            * (
                end - control
            )
        )
    )


def tangent_heading(
    tangent,
):
    """
    Convert world_ned tangent vector to project heading.

    tangent[0] = North
    tangent[1] = East
    """

    north = float(
        tangent[0]
    )

    east = float(
        tangent[1]
    )

    magnitude = math.hypot(
        north,
        east,
    )

    if magnitude <= 1e-8:

        raise ValueError(
            "Curve tangent is undefined. "
            "Choose different Start, Apogee, "
            "and End points."
        )

    return math.atan2(
        east,
        north,
    )


def sample_curve(
    start,
    apogee,
    end,
    step=DEFAULT_SAMPLE_STEP,
):
    """
    Sample the exact three-point quadratic curve.

    Returns:
        path [N,2]
        start_heading
        end_heading
        control_point
    """

    if step <= 0.0:

        raise ValueError(
            "step must be > 0"
        )

    start = _point(
        start
    )

    apogee = _point(
        apogee
    )

    end = _point(
        end
    )

    if (
        np.linalg.norm(
            end - start
        )
        <= 1e-6
    ):

        raise ValueError(
            "Curve Start and End must "
            "be different points."
        )

    control = curve_control_point(
        start,
        apogee,
        end,
    )

    start_tangent = curve_tangent(
        start,
        control,
        end,
        0.0,
    )

    end_tangent = curve_tangent(
        start,
        control,
        end,
        1.0,
    )

    start_heading = tangent_heading(
        start_tangent
    )

    end_heading = tangent_heading(
        end_tangent
    )

    # First estimate curve length densely.
    preview_t = np.linspace(
        0.0,
        1.0,
        1001,
    )

    preview = evaluate_curve(
        start,
        control,
        end,
        preview_t,
    )

    differences = (
        preview[1:]
        - preview[:-1]
    )

    estimated_length = float(
        np.sum(
            np.hypot(
                differences[:, 0],
                differences[:, 1],
            )
        )
    )

    if estimated_length <= 1e-6:

        raise ValueError(
            "Generated curve has near-zero length."
        )

    sample_count = max(
        2,
        int(
            math.ceil(
                estimated_length
                / float(step)
            )
        ),
    )

    t_values = np.linspace(
        0.0,
        1.0,
        sample_count + 1,
    )

    path = evaluate_curve(
        start,
        control,
        end,
        t_values,
    )

    # Numerical guarantee that the selected points are exact.
    path[0] = start
    path[-1] = end

    # t = 0.5 is the user-selected apogee by construction.
    apogee_check = evaluate_curve(
        start,
        control,
        end,
        np.asarray(
            0.5
        ),
    )

    if (
        np.linalg.norm(
            apogee_check
            - apogee
        )
        > 1e-7
    ):

        raise RuntimeError(
            "Curve interpolation failed to "
            "pass through the selected apogee."
        )

    return (
        path,
        start_heading,
        end_heading,
        control,
    )


def validate_curve(
    path,
    safe_grid,
    clearance,
    metadata,
):
    """
    Validate the complete sampled curve against the safe grid.
    """

    return validate_path_on_safe_grid(
        path,
        safe_grid,
        clearance,
        metadata,
    )


def build_curve_mission_plan(
    selected_start,
    selected_apogee,
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
    Build the complete R1 curve mission.
    """

    selected_start = (
        float(
            selected_start[0]
        ),
        float(
            selected_start[1]
        ),
    )

    selected_apogee = (
        float(
            selected_apogee[0]
        ),
        float(
            selected_apogee[1]
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
    # 1. Generate exact user-defined curve
    # ========================================================

    (
        curve_path,
        curve_start_heading,
        curve_end_heading,
        control_point,
    ) = sample_curve(
        selected_start,
        selected_apogee,
        selected_end,
        step=DEFAULT_SAMPLE_STEP,
    )

    # ========================================================
    # 2. Validate ENTIRE curve
    # ========================================================

    curve_validation = (
        validate_curve(
            curve_path,
            safe_grid,
            clearance,
            metadata,
        )
    )

    if not curve_validation[
        "safe"
    ]:

        raise RuntimeError(
            "Selected START -> APOGEE -> END curve "
            "crosses terrain or the WAM-V "
            "safety buffer."
        )

    # ========================================================
    # 3. Bring R1 safely to curve START and align it
    #    with the curve's starting tangent.
    # ========================================================

    (
        approach,
        approach_grid,
        approach_validation,
    ) = build_mission_approach(
        start_ned=r1_start_ned,
        start_heading=r1_start_heading,
        target_ned=selected_start,
        target_heading=(
            curve_start_heading
        ),
        turning_radius=(
            approach_turn_radius
        ),
        safe_grid=safe_grid,
        clearance=clearance,
        metadata=metadata,
        raw_grid=raw_grid,
    )

    # ========================================================
    # 4. Join approach + exact curve
    # ========================================================

    complete_path = np.vstack(
        (
            approach[
                :,
                :2
            ],

            curve_path[
                1:
            ],
        )
    )

    # ========================================================
    # 5. Validate complete mission after any special
    #    spawn-buffer escape has ended.
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
            "Combined R1 approach + curve mission "
            "failed collision validation."
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
    # 6. Curve metric length
    # ========================================================

    curve_differences = (
        curve_path[1:]
        - curve_path[:-1]
    )

    curve_length = float(
        np.sum(
            np.hypot(
                curve_differences[:, 0],
                curve_differences[:, 1],
            )
        )
    )

    # ========================================================
    # 7. Common MissionPlan
    # ========================================================

    mission = MissionPlan(
        mission_type="curve",

        path_points=[
            (
                float(point[0]),
                float(point[1]),
            )
            for point
            in complete_path
        ],

        parameters={
            "curve_start":
                selected_start,

            "curve_apogee":
                selected_apogee,

            "curve_end":
                selected_end,

            "curve_start_heading":
                float(
                    curve_start_heading
                ),

            "curve_end_heading":
                float(
                    curve_end_heading
                ),

            "control_point":
                (
                    float(
                        control_point[0]
                    ),
                    float(
                        control_point[1]
                    ),
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
            "curve_length":
                curve_length,

            "curve_samples":
                int(
                    len(
                        curve_path
                    )
                ),

            "approach_samples":
                int(
                    len(
                        approach
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

            "curve_minimum_clearance":
                float(
                    curve_validation[
                        "minimum_clearance"
                    ]
                ),
        },
    )

    mission.validate()

    return mission
