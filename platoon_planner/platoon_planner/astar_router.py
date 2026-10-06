#!/usr/bin/env python3

"""
Grid-based A* routing for the Sydney Regatta environment.

The router operates entirely in the project's world_ned convention:

    x = North
    y = East

It uses the already-inflated safe occupancy map, meaning cells that are
technically water but too close to terrain for the WAM-V are treated as
blocked.

The route cost also includes a soft clearance preference so the search
can prefer open water instead of unnecessarily hugging the minimum
shoreline boundary.
"""

import heapq
import math


SQRT2 = math.sqrt(2.0)


def _octile_distance(a, b):
    """Admissible heuristic for an 8-connected grid."""
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])

    diagonal = min(dr, dc)
    straight = max(dr, dc) - diagonal

    return diagonal * SQRT2 + straight


def _reconstruct_path(came_from, current):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar_grid(
    safe_occupancy,
    clearance,
    start,
    goal,
    resolution,
    preferred_clearance=12.0,
    clearance_weight=1.5,
):
    """
    Find an 8-connected safe route.

    Args:
        safe_occupancy:
            0 = navigable
            1 = unsafe

        clearance:
            terrain-clearance map in metres

        start:
            (row, column)

        goal:
            (row, column)

        resolution:
            grid resolution in metres

        preferred_clearance:
            soft preference threshold. Clearance above this value
            receives no additional cost.

        clearance_weight:
            strength of the soft low-clearance penalty.

    Returns:
        list[(row, column)]

    Raises:
        ValueError:
            invalid start / goal

        RuntimeError:
            no route exists
    """

    rows, columns = safe_occupancy.shape

    def in_bounds(cell):
        row, column = cell

        return (
            0 <= row < rows
            and
            0 <= column < columns
        )

    if not in_bounds(start):
        raise ValueError(
            f"Start cell outside map: {start}"
        )

    if not in_bounds(goal):
        raise ValueError(
            f"Goal cell outside map: {goal}"
        )

    if safe_occupancy[start] != 0:
        raise ValueError(
            f"Start cell is unsafe: {start}"
        )

    if safe_occupancy[goal] != 0:
        raise ValueError(
            f"Goal cell is unsafe: {goal}"
        )

    if preferred_clearance <= 0.0:
        raise ValueError(
            "preferred_clearance must be > 0"
        )

    if clearance_weight < 0.0:
        raise ValueError(
            "clearance_weight must be >= 0"
        )

    neighbours = (
        (-1,  0, 1.0),
        ( 1,  0, 1.0),
        ( 0, -1, 1.0),
        ( 0,  1, 1.0),

        (-1, -1, SQRT2),
        (-1,  1, SQRT2),
        ( 1, -1, SQRT2),
        ( 1,  1, SQRT2),
    )

    open_heap = []

    g_score = {
        start: 0.0
    }

    came_from = {}

    start_h = (
        _octile_distance(
            start,
            goal,
        )
        * resolution
    )

    heapq.heappush(
        open_heap,
        (
            start_h,
            0.0,
            start,
        ),
    )

    closed = set()

    while open_heap:
        _, current_g, current = heapq.heappop(
            open_heap
        )

        if current in closed:
            continue

        if current == goal:
            return _reconstruct_path(
                came_from,
                current,
            )

        closed.add(current)

        row, column = current

        for dr, dc, step_scale in neighbours:
            next_row = row + dr
            next_column = column + dc

            neighbour = (
                next_row,
                next_column,
            )

            if not in_bounds(neighbour):
                continue

            if safe_occupancy[neighbour] != 0:
                continue

            # Prevent diagonal corner cutting.
            if dr != 0 and dc != 0:
                side_a = (
                    row + dr,
                    column,
                )

                side_b = (
                    row,
                    column + dc,
                )

                if (
                    safe_occupancy[side_a] != 0
                    or
                    safe_occupancy[side_b] != 0
                ):
                    continue

            local_clearance = float(
                clearance[neighbour]
            )

            clearance_deficit = max(
                0.0,
                (
                    preferred_clearance
                    - local_clearance
                )
                / preferred_clearance,
            )

            clearance_penalty = (
                clearance_weight
                * clearance_deficit
                * clearance_deficit
            )

            step_distance = (
                step_scale
                * resolution
            )

            step_cost = (
                step_distance
                * (
                    1.0
                    + clearance_penalty
                )
            )

            tentative_g = (
                current_g
                + step_cost
            )

            if tentative_g >= g_score.get(
                neighbour,
                float("inf"),
            ):
                continue

            came_from[neighbour] = current
            g_score[neighbour] = tentative_g

            heuristic = (
                _octile_distance(
                    neighbour,
                    goal,
                )
                * resolution
            )

            heapq.heappush(
                open_heap,
                (
                    tentative_g + heuristic,
                    tentative_g,
                    neighbour,
                ),
            )

    raise RuntimeError(
        f"No safe A* route from {start} to {goal}"
    )


def grid_path_to_ned(
    grid_path,
    metadata,
):
    """
    Convert an A* cell route to world_ned cell-centre coordinates.
    """
    from platoon_planner.sydney_environment import grid_to_ned

    return [
        grid_to_ned(
            row,
            column,
            metadata,
        )
        for row, column in grid_path
    ]


def path_length_ned(path):
    """Calculate planar metric path length."""
    if len(path) < 2:
        return 0.0

    total = 0.0

    for first, second in zip(
        path[:-1],
        path[1:],
    ):
        total += math.hypot(
            second[0] - first[0],
            second[1] - first[1],
        )

    return total


# ============================================================
# SAFE A* PATH SIMPLIFICATION
# ============================================================

def supercover_line_cells(start, goal):
    """
    Return every grid cell touched by the straight line between two
    cell centres.

    This is deliberately more conservative than ordinary Bresenham:
    when the line passes exactly through a grid corner, both adjacent
    side cells are included as well.

    Cells are returned as:
        (row, column)
    """

    row0, col0 = start
    row1, col1 = goal

    x0 = int(col0)
    y0 = int(row0)

    x1 = int(col1)
    y1 = int(row1)

    dx = x1 - x0
    dy = y1 - y0

    nx = abs(dx)
    ny = abs(dy)

    sign_x = (
        1 if dx > 0
        else -1 if dx < 0
        else 0
    )

    sign_y = (
        1 if dy > 0
        else -1 if dy < 0
        else 0
    )

    x = x0
    y = y0

    ix = 0
    iy = 0

    cells = []
    seen = set()

    def add_cell(row, column):
        cell = (
            int(row),
            int(column),
        )

        if cell not in seen:
            seen.add(cell)
            cells.append(cell)

    add_cell(
        y,
        x,
    )

    while (
        ix < nx
        or
        iy < ny
    ):

        decision = (
            (1 + 2 * ix) * ny
            -
            (1 + 2 * iy) * nx
        )

        if decision == 0:

            old_x = x
            old_y = y

            x += sign_x
            y += sign_y

            ix += 1
            iy += 1

            # Exact corner crossing:
            # conservatively include both side cells.
            if sign_x != 0:
                add_cell(
                    old_y,
                    x,
                )

            if sign_y != 0:
                add_cell(
                    y,
                    old_x,
                )

            add_cell(
                y,
                x,
            )

        elif decision < 0:

            x += sign_x
            ix += 1

            add_cell(
                y,
                x,
            )

        else:

            y += sign_y
            iy += 1

            add_cell(
                y,
                x,
            )

    return cells


def line_of_sight_safe(
    safe_occupancy,
    start,
    goal,
):
    """
    Check whether a straight grid-space segment remains entirely
    inside WAM-V-safe navigable water.
    """

    rows, columns = safe_occupancy.shape

    cells = supercover_line_cells(
        start,
        goal,
    )

    for row, column in cells:

        if not (
            0 <= row < rows
            and
            0 <= column < columns
        ):
            return False

        if safe_occupancy[
            row,
            column
        ] != 0:
            return False

    return True


def simplify_grid_path(
    grid_path,
    safe_occupancy,
):
    """
    Reduce a dense A* route to a small set of safe waypoints.

    Starting at each retained waypoint, find the farthest later A*
    waypoint that can be connected by a collision-free straight line.

    The first and final A* cells are always preserved.
    """

    if not grid_path:
        return []

    if len(grid_path) <= 2:
        return list(
            grid_path
        )

    simplified = [
        grid_path[0]
    ]

    anchor_index = 0
    final_index = (
        len(grid_path) - 1
    )

    while anchor_index < final_index:

        candidate_index = (
            final_index
        )

        while (
            candidate_index
            > anchor_index + 1
        ):

            if line_of_sight_safe(
                safe_occupancy,
                grid_path[
                    anchor_index
                ],
                grid_path[
                    candidate_index
                ],
            ):
                break

            candidate_index -= 1

        # Adjacent A* cells should always be safely connected.
        if candidate_index <= anchor_index:
            raise RuntimeError(
                "A* simplification could not advance "
                f"from path index {anchor_index}"
            )

        simplified.append(
            grid_path[
                candidate_index
            ]
        )

        anchor_index = (
            candidate_index
        )

    return simplified
