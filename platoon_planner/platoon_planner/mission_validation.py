#!/usr/bin/env python3

"""
Common validation for complete R1 missions.

Supports the normal safe grid plus the known Sydney spawn case where
R1 may initially sit slightly inside the additional planning buffer
while still being physically clear of terrain.
"""

from platoon_planner.sydney_environment import (
    ned_to_grid,
    wamv_gps_footprint_radius,
)

from platoon_planner.astar_router import (
    supercover_line_cells,
)

from platoon_planner.dubins_router import (
    validate_path_on_safe_grid,
)


def validate_complete_mission(
    path_points,
    raw_grid,
    safe_grid,
    clearance,
    metadata,
):
    if len(path_points) < 2:
        raise ValueError(
            "Mission requires at least two path points."
        )

    cells = [
        ned_to_grid(
            point[0],
            point[1],
            metadata,
        )
        for point in path_points
    ]

    # ---------------------------------------------------------
    # Find first ordinary safe-grid sample.
    # ---------------------------------------------------------

    safe_start_index = None

    for index, cell in enumerate(cells):

        row, column = cell

        if not (
            0 <= row < safe_grid.shape[0]
            and
            0 <= column < safe_grid.shape[1]
        ):
            raise RuntimeError(
                "Mission leaves the navigation map."
            )

        if safe_grid[cell] == 0:
            safe_start_index = index
            break

    if safe_start_index is None:
        raise RuntimeError(
            "Mission never enters ordinary safe water."
        )

    physical_clearance = (
        wamv_gps_footprint_radius()
    )

    minimum_clearance = float("inf")

    # ---------------------------------------------------------
    # Validate any initial spawn-buffer escape physically.
    # ---------------------------------------------------------

    previous_cell = None

    for index in range(
        safe_start_index + 1
    ):

        cell = cells[index]

        if previous_cell is None:
            crossed = [cell]
        else:
            crossed = supercover_line_cells(
                previous_cell,
                cell,
            )

        for crossed_cell in crossed:

            row, column = crossed_cell

            if not (
                0 <= row < raw_grid.shape[0]
                and
                0 <= column < raw_grid.shape[1]
            ):
                raise RuntimeError(
                    "Initial mission segment leaves map."
                )

            if raw_grid[crossed_cell] != 0:
                raise RuntimeError(
                    "Initial mission segment crosses "
                    "collision terrain."
                )

            local_clearance = float(
                clearance[crossed_cell]
            )

            if (
                local_clearance
                < physical_clearance
            ):
                raise RuntimeError(
                    "Initial mission segment does not "
                    "have enough clearance for the "
                    "physical WAM-V footprint."
                )

            minimum_clearance = min(
                minimum_clearance,
                local_clearance,
            )

        previous_cell = cell

    # ---------------------------------------------------------
    # From safe water onward, enforce full planning clearance.
    # ---------------------------------------------------------

    strict_validation = (
        validate_path_on_safe_grid(
            path_points[
                safe_start_index:
            ],
            safe_grid,
            clearance,
            metadata,
        )
    )

    if not strict_validation["safe"]:
        return {
            **strict_validation,
            "safe_start_index":
                safe_start_index,

            "spawn_escape_used":
                safe_start_index > 0,
        }

    minimum_clearance = min(
        minimum_clearance,
        float(
            strict_validation[
                "minimum_clearance"
            ]
        ),
    )

    return {
        "safe":
            True,

        "reason":
            (
                "ok_with_spawn_escape"
                if safe_start_index > 0
                else "ok"
            ),

        "minimum_clearance":
            minimum_clearance,

        "safe_start_index":
            safe_start_index,

        "spawn_escape_used":
            safe_start_index > 0,
    }
