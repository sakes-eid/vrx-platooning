#!/usr/bin/env python3

"""
Generic safe approach routing from R1 to a mission start point.

Normal pipeline:

    R1 position
        -> A*
        -> safe A* simplification
        -> Dubins smoothing
        -> coverage start
        -> collision validation

Spawn-buffer exception:

    If R1 initially spawns in the inflated planning buffer, but:
        - it is NOT on raw terrain, and
        - it still has enough clearance for the physical WAM-V footprint,

    the planner permits one short escape from the real spawn position
    to the nearest normal safe-water cell.

    Once that safe cell is reached, all ordinary safe-grid constraints
    apply again.
"""

import math

import numpy as np

from platoon_planner.sydney_environment import (
    ned_to_grid,
    grid_to_ned,
    wamv_gps_footprint_radius,
)

from platoon_planner.astar_router import (
    astar_grid,
    simplify_grid_path,
    grid_path_to_ned,
    supercover_line_cells,
)

from platoon_planner.dubins_router import (
    generate_dubins_via_waypoints,
    validate_path_on_safe_grid,
    heading_between,
)


def _sample_straight(
    start,
    end,
    step=0.5,
):
    """
    Sample a straight NED segment.
    """

    dn = (
        float(end[0])
        - float(start[0])
    )

    de = (
        float(end[1])
        - float(start[1])
    )

    length = math.hypot(
        dn,
        de,
    )

    count = max(
        1,
        int(
            math.ceil(
                length / step
            )
        ),
    )

    samples = []

    heading = math.atan2(
        de,
        dn,
    )

    for index in range(
        count + 1
    ):

        ratio = (
            index / count
        )

        samples.append(
            (
                float(start[0])
                + ratio * dn,

                float(start[1])
                + ratio * de,

                heading,
            )
        )

    return np.asarray(
        samples,
        dtype=float,
    )


def _nearest_safe_escape_cell(
    start_ned,
    start_cell,
    safe_grid,
    raw_grid,
    clearance,
    metadata,
    minimum_target_clearance,
    max_radius_cells=25,
):
    """
    Find a nearby ordinary-safe cell reachable from the spawn while
    remaining physically clear of raw terrain.

    The temporary escape may use cells inside the additional planning
    margin, but never cells that violate the physical WAM-V footprint.
    """

    physical_clearance = (
        wamv_gps_footprint_radius()
    )

    start_row, start_col = (
        start_cell
    )

    for radius in range(
        1,
        max_radius_cells + 1,
    ):

        candidates = []

        row_min = max(
            0,
            start_row - radius,
        )

        row_max = min(
            safe_grid.shape[0] - 1,
            start_row + radius,
        )

        col_min = max(
            0,
            start_col - radius,
        )

        col_max = min(
            safe_grid.shape[1] - 1,
            start_col + radius,
        )

        for row in range(
            row_min,
            row_max + 1,
        ):

            for col in range(
                col_min,
                col_max + 1,
            ):

                # Only inspect the current search ring.
                if max(
                    abs(
                        row - start_row
                    ),
                    abs(
                        col - start_col
                    ),
                ) != radius:
                    continue

                if safe_grid[
                    row,
                    col,
                ] != 0:
                    continue

                # Do not begin the Dubins maneuver immediately
                # after barely entering the ordinary safe grid.
                # The vessel first escapes deeper into open water
                # so the initial turn has room to develop.
                if (
                    float(
                        clearance[
                            row,
                            col,
                        ]
                    )
                    < float(
                        minimum_target_clearance
                    )
                ):
                    continue

                north, east = (
                    grid_to_ned(
                        row,
                        col,
                        metadata,
                    )
                )

                distance = math.hypot(
                    north
                    - float(
                        start_ned[0]
                    ),
                    east
                    - float(
                        start_ned[1]
                    ),
                )

                candidates.append(
                    (
                        distance,
                        (
                            row,
                            col,
                        ),
                    )
                )

        candidates.sort(
            key=lambda item: item[0]
        )

        for _, candidate in candidates:

            crossed_cells = (
                supercover_line_cells(
                    start_cell,
                    candidate,
                )
            )

            valid = True

            for cell in crossed_cells:

                row, col = cell

                if raw_grid[
                    row,
                    col,
                ] != 0:

                    valid = False
                    break

                if (
                    float(
                        clearance[
                            row,
                            col,
                        ]
                    )
                    <
                    physical_clearance
                ):

                    valid = False
                    break

            if valid:
                return candidate

    raise RuntimeError(
        "R1 spawned inside the planning buffer, "
        "but no physically safe escape to normal "
        "safe water was found."
    )


def build_mission_approach(
    start_ned,
    start_heading,
    target_ned,
    target_heading,
    turning_radius,
    safe_grid,
    clearance,
    metadata,
    raw_grid=None,
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

    turn_radius = float(
        turning_radius
    )

    if turn_radius <= 0.0:
        raise ValueError(
            "turning_radius must be > 0"
        )

    original_start_cell = (
        ned_to_grid(
            start_ned[0],
            start_ned[1],
            metadata,
        )
    )

    goal_cell = ned_to_grid(
        float(target_ned[0]),
        float(target_ned[1]),
        metadata,
    )

    row, column = (
        original_start_cell
    )

    if not (
        0
        <= row
        < safe_grid.shape[0]
        and
        0
        <= column
        < safe_grid.shape[1]
    ):

        raise RuntimeError(
            "R1 start position is outside "
            "the navigation map."
        )

    escape_used = False
    escape_samples = None
    escape_min_clearance = None

    # ========================================================
    # Normal safe start
    # ========================================================

    if safe_grid[
        original_start_cell
    ] == 0:

        astar_start_cell = (
            original_start_cell
        )

        astar_start_ned = (
            float(
                start_ned[0]
            ),
            float(
                start_ned[1]
            ),
        )

        dubins_start_heading = float(
            start_heading
        )

    # ========================================================
    # Spawn-buffer escape
    # ========================================================

    else:

        if raw_grid is None:

            raise RuntimeError(
                "R1 start is inside the planning "
                "safety buffer and raw_grid was not "
                "provided for spawn-escape validation."
            )

        # The spawn itself must still be actual water.
        if raw_grid[
            original_start_cell
        ] != 0:

            raise RuntimeError(
                "R1 start position is on raw "
                "collision terrain."
            )

        physical_clearance = (
            wamv_gps_footprint_radius()
        )

        start_clearance = float(
            clearance[
                original_start_cell
            ]
        )

        if (
            start_clearance
            < physical_clearance
        ):

            raise RuntimeError(
                "R1 start position does not have "
                "enough clearance for the physical "
                "WAM-V footprint."
            )

        # The normal safe boundary is enough for collision
        # avoidance, but not necessarily enough room to begin
        # a finite-radius turn.  Escape farther into open water
        # before enabling A* + Dubins.
        maneuver_clearance = max(
            12.0,
            2.0 * turn_radius,
        )

        astar_start_cell = (
            _nearest_safe_escape_cell(
                start_ned=start_ned,
                start_cell=(
                    original_start_cell
                ),
                safe_grid=safe_grid,
                raw_grid=raw_grid,
                clearance=clearance,
                metadata=metadata,
                minimum_target_clearance=(
                    maneuver_clearance
                ),
            )
        )

        astar_start_ned = (
            grid_to_ned(
                astar_start_cell[0],
                astar_start_cell[1],
                metadata,
            )
        )

        escape_samples = (
            _sample_straight(
                start_ned,
                astar_start_ned,
                step=0.5,
            )
        )

        escape_cells = (
            supercover_line_cells(
                original_start_cell,
                astar_start_cell,
            )
        )

        escape_min_clearance = min(
            float(
                clearance[
                    cell
                ]
            )
            for cell
            in escape_cells
        )

        dubins_start_heading = (
            heading_between(
                start_ned,
                astar_start_ned,
            )
        )

        escape_used = True

    # ========================================================
    # A*
    # ========================================================

    astar_path = astar_grid(
        safe_grid,
        clearance,
        astar_start_cell,
        goal_cell,
        resolution=metadata[
            "resolution"
        ],
        preferred_clearance=max(
            12.0,
            2.0 * turn_radius,
        ),
        clearance_weight=1.5,
    )

    # ========================================================
    # Simplify A*
    # ========================================================

    simplified = simplify_grid_path(
        astar_path,
        safe_grid,
    )

    route_points = grid_path_to_ned(
        simplified,
        metadata,
    )

    route_points[0] = (
        float(
            astar_start_ned[0]
        ),
        float(
            astar_start_ned[1]
        ),
    )

    route_points[-1] = (
        float(target_ned[0]),
        float(target_ned[1]),
    )

    # ========================================================
    # Dubins smoothing from NORMAL safe water onward
    # ========================================================

    approach_core, _ = (
        generate_dubins_via_waypoints(
            route_points,
            start_heading=(
                dubins_start_heading
            ),
            goal_heading=float(
                target_heading
            ),
            turning_radius=(
                turn_radius
            ),
            step=0.5,
        )
    )

    core_validation = (
        validate_path_on_safe_grid(
            approach_core,
            safe_grid,
            clearance,
            metadata,
        )
    )

    if not core_validation[
        "safe"
    ]:

        raise RuntimeError(
            "A* found a route to the coverage path, "
            "but the Dubins-smoothed approach failed "
            "collision validation."
        )

    # ========================================================
    # Join temporary escape + normal safe approach
    # ========================================================

    if escape_used:

        approach = np.vstack(
            (
                escape_samples,
                approach_core[1:],
            )
        )

        safe_start_index = (
            len(
                escape_samples
            )
            - 1
        )

        minimum_clearance = min(
            escape_min_clearance,
            float(
                core_validation[
                    "minimum_clearance"
                ]
            ),
        )

        validation = dict(
            core_validation
        )

        validation.update(
            {
                "safe":
                    True,

                "reason":
                    "ok_with_spawn_escape",

                "escape_used":
                    True,

                "safe_start_index":
                    int(
                        safe_start_index
                    ),

                "spawn_clearance":
                    float(
                        clearance[
                            original_start_cell
                        ]
                    ),

                "minimum_clearance":
                    float(
                        minimum_clearance
                    ),
            }
        )

    else:

        approach = (
            approach_core
        )

        validation = dict(
            core_validation
        )

        validation.update(
            {
                "escape_used":
                    False,

                "safe_start_index":
                    0,

                "spawn_clearance":
                    float(
                        clearance[
                            original_start_cell
                        ]
                    ),
            }
        )

    return (
        approach,
        simplified,
        validation,
    )
