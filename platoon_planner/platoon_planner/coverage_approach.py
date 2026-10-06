#!/usr/bin/env python3

"""
Safe approach routing from R1 to the first coverage lane.

Pipeline:

    R1 position
        -> A*
        -> safe A* simplification
        -> Dubins smoothing
        -> coverage start
        -> collision validation
"""

from platoon_planner.sydney_environment import (
    ned_to_grid,
)

from platoon_planner.astar_router import (
    astar_grid,
    simplify_grid_path,
    grid_path_to_ned,
)

from platoon_planner.dubins_router import (
    generate_dubins_via_waypoints,
    validate_path_on_safe_grid,
)


def build_approach_path(
    start_ned,
    start_heading,
    first_segment,
    line_spacing,
    safe_grid,
    clearance,
    metadata,
):
    """
    Build a safe path from R1 to the beginning of the first
    coverage segment.

    start_ned:
        (North, East)

    start_heading:
        world_ned heading in radians

    first_segment:
        first coverage-segment dictionary
    """

    # Coverage line spacing defines the turn diameter.
    turn_radius = (
        float(line_spacing)
        / 2.0
    )

    start_cell = ned_to_grid(
        start_ned[0],
        start_ned[1],
        metadata,
    )

    goal_cell = (
        first_segment[
            "start_cell"
        ]
    )

    # --------------------------------------------------------
    # Validate R1 start
    # --------------------------------------------------------

    row, column = start_cell

    if not (
        0 <= row < safe_grid.shape[0]
        and
        0 <= column < safe_grid.shape[1]
    ):
        raise RuntimeError(
            "R1 start position is outside the navigation map."
        )

    if safe_grid[
        start_cell
    ] != 0:

        raise RuntimeError(
            "R1 start position is not in safe water."
        )

    # --------------------------------------------------------
    # A*
    # --------------------------------------------------------

    astar_path = astar_grid(
        safe_grid,
        clearance,
        start_cell,
        goal_cell,
        resolution=metadata[
            "resolution"
        ],
        preferred_clearance=max(
            12.0,
            float(line_spacing),
        ),
        clearance_weight=1.5,
    )

    # --------------------------------------------------------
    # Simplify A*
    # --------------------------------------------------------

    simplified = simplify_grid_path(
        astar_path,
        safe_grid,
    )

    route_points = grid_path_to_ned(
        simplified,
        metadata,
    )

    # Replace grid-cell centres with the exact R1 and coverage
    # coordinates.
    route_points[0] = (
        float(start_ned[0]),
        float(start_ned[1]),
    )

    route_points[-1] = (
        first_segment[
            "start_ned"
        ]
    )

    # --------------------------------------------------------
    # Dubins smoothing
    # --------------------------------------------------------

    approach, _ = generate_dubins_via_waypoints(
        route_points,
        start_heading=float(
            start_heading
        ),
        goal_heading=float(
            first_segment[
                "heading"
            ]
        ),
        turning_radius=turn_radius,
        step=0.5,
    )

    # --------------------------------------------------------
    # Final safety validation
    # --------------------------------------------------------

    validation = validate_path_on_safe_grid(
        approach,
        safe_grid,
        clearance,
        metadata,
    )

    if not validation[
        "safe"
    ]:

        raise RuntimeError(
            "A* found a route to the coverage path, "
            "but the Dubins-smoothed approach failed "
            "collision validation."
        )

    return (
        approach,
        simplified,
        validation,
    )
