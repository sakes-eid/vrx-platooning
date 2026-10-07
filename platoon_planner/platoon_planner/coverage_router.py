#!/usr/bin/env python3

"""
Interactive, map-aware Sydney Regatta coverage planner.

Workflow:

    1. Display the 3D-aware WAM-V-safe map.
    2. User right-clicks two opposite corners.
       The FIRST click is also the preferred starting corner.
    3. User enters coverage line spacing.
    4. Safe north/south coverage lanes are extracted.
    5. Terrain automatically splits unsafe lane sections.
    6. Safe direct Dubins transitions are attempted first.
    7. If blocked, A* finds a safe connector and Dubins smooths it.
    8. The complete mission is collision validated.

Coordinates:
    world_ned
    x = North
    y = East
"""

import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

from platoon_planner.sydney_environment import (
    find_sydney_mesh,
    load_shore_vertices,
    load_shore_triangles,
    build_safe_occupancy,
    wamv_navigation_vertical_band,
    build_vertical_slab_occupancy_grid,
    grid_to_ned,
    ned_to_grid,
)

from platoon_planner.astar_router import (
    astar_grid,
    simplify_grid_path,
    grid_path_to_ned,
)

from platoon_planner.vehicle_state_source import (
    get_vehicle_state_once,
)

from platoon_planner.coverage_approach import (
    build_approach_path,
)

from platoon_planner.dubins_router import (
    generate_dubins_path,
    generate_dubins_via_waypoints,
    validate_path_on_safe_grid,
    path_length,
)


# ============================================================
# DISPLAY / SELECTION
# ============================================================

def display_map(
    raw_grid,
    safe_grid,
):
    """
    0 = safe water
    1 = safety buffer
    2 = collision-relevant geometry
    """

    display = np.zeros_like(
        raw_grid
    )

    display[
        (safe_grid == 1)
        & (raw_grid == 0)
    ] = 1

    display[
        raw_grid == 1
    ] = 2

    return display


def select_coverage_rectangle(
    raw_grid,
    safe_grid,
    metadata,
):
    """
    Interactive selection matching the useful behaviour of the
    planner-team interface.

    Left drag:
        pan

    Mouse wheel:
        zoom

    Right click:
        select two opposite rectangle corners

    The first selected corner also defines the preferred mission
    starting side/direction.
    """

    display = display_map(
        raw_grid,
        safe_grid,
    )

    cmap = ListedColormap([
        "white",
        "orange",
        "black",
    ])

    fig, ax = plt.subplots(
        figsize=(15, 9)
    )

    ax.imshow(
        display,
        origin="lower",
        extent=[
            metadata["east_min"],
            metadata["east_max"],
            metadata["north_min"],
            metadata["north_max"],
        ],
        interpolation="nearest",
        aspect="equal",
        cmap=cmap,
        vmin=0,
        vmax=2,
    )

    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")

    ax.set_title(
        "Coverage Selection\n"
        "Right-click TWO opposite corners | "
        "First click = preferred start corner"
    )

    selected = []
    drag_state = None


    def on_press(event):

        nonlocal drag_state

        if event.inaxes != ax:
            return

        # Left drag = pan.
        if event.button == 1:

            drag_state = (
                event.x,
                event.y,
                ax.get_xlim(),
                ax.get_ylim(),
            )

            return

        # Right click = coverage corner.
        if event.button != 3:
            return

        if (
            event.xdata is None
            or
            event.ydata is None
        ):
            return

        if len(selected) >= 2:
            return

        # Store as world_ned:
        # (North, East)
        point = (
            float(event.ydata),
            float(event.xdata),
        )

        selected.append(
            point
        )

        label = (
            "Start corner"
            if len(selected) == 1
            else "Opposite corner"
        )

        ax.scatter(
            point[1],
            point[0],
            s=80,
            marker="x",
            label=label,
        )

        print(
            f"{label}: "
            f"North={point[0]:.2f}, "
            f"East={point[1]:.2f}"
        )

        if len(selected) == 2:

            first = selected[0]
            second = selected[1]

            n_min = min(
                first[0],
                second[0],
            )

            n_max = max(
                first[0],
                second[0],
            )

            e_min = min(
                first[1],
                second[1],
            )

            e_max = max(
                first[1],
                second[1],
            )

            ax.plot(
                [
                    e_min,
                    e_max,
                    e_max,
                    e_min,
                    e_min,
                ],
                [
                    n_min,
                    n_min,
                    n_max,
                    n_max,
                    n_min,
                ],
                linestyle="--",
                linewidth=2.0,
                label="Coverage area",
            )

            ax.legend()
            fig.canvas.draw_idle()

            plt.pause(0.5)
            plt.close(fig)

        else:
            ax.legend()
            fig.canvas.draw_idle()


    def on_motion(event):

        nonlocal drag_state

        if (
            drag_state is None
            or event.inaxes != ax
        ):
            return

        if (
            event.x is None
            or event.y is None
        ):
            return

        (
            start_x,
            start_y,
            x_limits,
            y_limits,
        ) = drag_state

        x0, x1 = x_limits
        y0, y1 = y_limits

        dx_pixels = (
            event.x - start_x
        )

        dy_pixels = (
            event.y - start_y
        )

        east_per_pixel = (
            (x1 - x0)
            / ax.bbox.width
        )

        north_per_pixel = (
            (y1 - y0)
            / ax.bbox.height
        )

        ax.set_xlim(
            x0
            - dx_pixels
            * east_per_pixel,

            x1
            - dx_pixels
            * east_per_pixel,
        )

        ax.set_ylim(
            y0
            - dy_pixels
            * north_per_pixel,

            y1
            - dy_pixels
            * north_per_pixel,
        )

        fig.canvas.draw_idle()


    def on_release(event):

        nonlocal drag_state

        if event.button == 1:
            drag_state = None


    def on_scroll(event):

        if event.inaxes != ax:
            return

        if (
            event.xdata is None
            or
            event.ydata is None
        ):
            return

        x0, x1 = ax.get_xlim()
        y0, y1 = ax.get_ylim()

        factor = (
            0.85
            if event.button == "up"
            else 1.15
        )

        width = (
            x1 - x0
        )

        height = (
            y1 - y0
        )

        x_fraction = (
            (event.xdata - x0)
            / width
        )

        y_fraction = (
            (event.ydata - y0)
            / height
        )

        new_width = (
            width * factor
        )

        new_height = (
            height * factor
        )

        ax.set_xlim(
            event.xdata
            - x_fraction
            * new_width,

            event.xdata
            + (
                1.0 - x_fraction
            )
            * new_width,
        )

        ax.set_ylim(
            event.ydata
            - y_fraction
            * new_height,

            event.ydata
            + (
                1.0 - y_fraction
            )
            * new_height,
        )

        fig.canvas.draw_idle()


    fig.canvas.mpl_connect(
        "button_press_event",
        on_press,
    )

    fig.canvas.mpl_connect(
        "motion_notify_event",
        on_motion,
    )

    fig.canvas.mpl_connect(
        "button_release_event",
        on_release,
    )

    fig.canvas.mpl_connect(
        "scroll_event",
        on_scroll,
    )

    plt.show()

    if len(selected) != 2:
        raise RuntimeError(
            "Coverage area selection cancelled"
        )

    return (
        selected[0],
        selected[1],
    )


# ============================================================
# SAFE-WATER CONNECTIVITY
# ============================================================

def rectangle_grid_bounds(
    first,
    second,
    metadata,
    shape,
):

    n_min = min(
        first[0],
        second[0],
    )

    n_max = max(
        first[0],
        second[0],
    )

    e_min = min(
        first[1],
        second[1],
    )

    e_max = max(
        first[1],
        second[1],
    )

    a = ned_to_grid(
        n_min,
        e_min,
        metadata,
    )

    b = ned_to_grid(
        n_max,
        e_max,
        metadata,
    )

    row_min = max(
        0,
        min(
            a[0],
            b[0],
        ),
    )

    row_max = min(
        shape[0] - 1,
        max(
            a[0],
            b[0],
        ),
    )

    col_min = max(
        0,
        min(
            a[1],
            b[1],
        ),
    )

    col_max = min(
        shape[1] - 1,
        max(
            a[1],
            b[1],
        ),
    )

    return (
        row_min,
        row_max,
        col_min,
        col_max,
    )


def nearest_safe_cell(
    safe_grid,
    metadata,
    point,
    bounds,
):
    """
    Find the safe cell inside the selected rectangle nearest to the
    user's first click.
    """

    row_min, row_max, col_min, col_max = (
        bounds
    )

    target = ned_to_grid(
        point[0],
        point[1],
        metadata,
    )

    best = None
    best_distance = float("inf")

    for row in range(
        row_min,
        row_max + 1,
    ):

        for col in range(
            col_min,
            col_max + 1,
        ):

            if safe_grid[
                row,
                col
            ] != 0:
                continue

            distance = (
                (row - target[0]) ** 2
                + (col - target[1]) ** 2
            )

            if distance < best_distance:

                best_distance = (
                    distance
                )

                best = (
                    row,
                    col,
                )

    if best is None:
        raise RuntimeError(
            "Selected rectangle contains no safe water"
        )

    return best


def reachable_safe_water(
    safe_grid,
    start,
):
    """
    8-connected reachability using the same no-corner-cutting rule as
    our A* router.
    """

    reachable = np.zeros(
        safe_grid.shape,
        dtype=bool,
    )

    queue = deque([
        start
    ])

    reachable[
        start
    ] = True

    neighbours = (
        (-1,  0),
        ( 1,  0),
        ( 0, -1),
        ( 0,  1),
        (-1, -1),
        (-1,  1),
        ( 1, -1),
        ( 1,  1),
    )

    rows, columns = (
        safe_grid.shape
    )

    while queue:

        row, col = (
            queue.popleft()
        )

        for dr, dc in neighbours:

            nr = row + dr
            nc = col + dc

            if not (
                0 <= nr < rows
                and
                0 <= nc < columns
            ):
                continue

            if reachable[
                nr,
                nc
            ]:
                continue

            if safe_grid[
                nr,
                nc
            ] != 0:
                continue

            # Prevent diagonal corner cutting.
            if (
                dr != 0
                and dc != 0
            ):

                if (
                    safe_grid[
                        row + dr,
                        col
                    ] != 0
                    or
                    safe_grid[
                        row,
                        col + dc
                    ] != 0
                ):
                    continue

            reachable[
                nr,
                nc
            ] = True

            queue.append(
                (
                    nr,
                    nc,
                )
            )

    return reachable


# ============================================================
# COVERAGE SEGMENTS
# ============================================================

def free_runs(
    usable,
    column,
    row_min,
    row_max,
):

    runs = []
    start = None

    for row in range(
        row_min,
        row_max + 1,
    ):

        free = bool(
            usable[
                row,
                column
            ]
        )

        if free:

            if start is None:
                start = row

        elif start is not None:

            runs.append(
                (
                    start,
                    row - 1,
                )
            )

            start = None

    if start is not None:

        runs.append(
            (
                start,
                row_max,
            )
        )

    return runs


def generate_coverage_segments(
    safe_grid,
    reachable,
    metadata,
    first_corner,
    second_corner,
    line_spacing,
):
    """
    Generate environment-aware boustrophedon lane segments.

    The first corner decides:
        - which side of the rectangle is visited first
        - initial north/south direction
    """

    if line_spacing <= 0.0:
        raise ValueError(
            "Line spacing must be > 0"
        )

    bounds = rectangle_grid_bounds(
        first_corner,
        second_corner,
        metadata,
        safe_grid.shape,
    )

    (
        row_min,
        row_max,
        col_min,
        col_max,
    ) = bounds

    usable = (
        (safe_grid == 0)
        & reachable
    )

    resolution = float(
        metadata["resolution"]
    )

    lane_step = max(
        1,
        int(
            round(
                line_spacing
                / resolution
            )
        ),
    )

    centre_north = (
        first_corner[0]
        + second_corner[0]
    ) / 2.0

    centre_east = (
        first_corner[1]
        + second_corner[1]
    ) / 2.0

    # Which East/West side did the user click first?
    start_from_west = (
        first_corner[1]
        <= centre_east
    )

    # Which North/South side did the user click first?
    first_northbound = (
        first_corner[0]
        <= centre_north
    )

    if start_from_west:

        columns = list(
            range(
                col_min,
                col_max + 1,
                lane_step,
            )
        )

    else:

        columns = list(
            range(
                col_max,
                col_min - 1,
                -lane_step,
            )
        )

    segments = []

    for lane_index, column in enumerate(
        columns
    ):

        northbound = (
            first_northbound
            if lane_index % 2 == 0
            else not first_northbound
        )

        runs = free_runs(
            usable,
            column,
            row_min,
            row_max,
        )

        if not northbound:
            runs = list(
                reversed(
                    runs
                )
            )

        for run_start, run_end in runs:

            # Require at least two safe cells.
            if run_end <= run_start:
                continue

            if northbound:

                start_cell = (
                    run_start,
                    column,
                )

                end_cell = (
                    run_end,
                    column,
                )

                heading = 0.0
                direction = "north"

            else:

                start_cell = (
                    run_end,
                    column,
                )

                end_cell = (
                    run_start,
                    column,
                )

                heading = math.pi
                direction = "south"

            start_ned = grid_to_ned(
                start_cell[0],
                start_cell[1],
                metadata,
            )

            end_ned = grid_to_ned(
                end_cell[0],
                end_cell[1],
                metadata,
            )

            segments.append(
                {
                    "lane_index":
                        lane_index,

                    "direction":
                        direction,

                    "heading":
                        heading,

                    "start_cell":
                        start_cell,

                    "end_cell":
                        end_cell,

                    "start_ned":
                        start_ned,

                    "end_ned":
                        end_ned,
                }
            )

    return (
        segments,
        bounds,
        lane_step,
    )


# ============================================================
# PATH HELPERS
# ============================================================

def sample_line(
    start,
    end,
    heading,
    spacing=0.5,
):

    distance = math.hypot(
        end[0] - start[0],
        end[1] - start[1],
    )

    count = max(
        1,
        int(
            math.ceil(
                distance
                / spacing
            )
        ),
    )

    samples = []

    for index in range(
        count + 1
    ):

        fraction = (
            index / count
        )

        samples.append(
            [
                start[0]
                + fraction
                * (
                    end[0]
                    - start[0]
                ),

                start[1]
                + fraction
                * (
                    end[1]
                    - start[1]
                ),

                heading,
            ]
        )

    return np.asarray(
        samples,
        dtype=float,
    )


def append_samples(
    destination,
    samples,
):

    if len(samples) == 0:
        return

    if not destination:

        destination.extend(
            samples.tolist()
        )

        return

    start_index = 0

    if math.hypot(
        destination[-1][0]
        - samples[0, 0],

        destination[-1][1]
        - samples[0, 1],
    ) < 1e-8:

        start_index = 1

    destination.extend(
        samples[
            start_index:
        ].tolist()
    )


# ============================================================
# SEGMENT CONNECTOR
# ============================================================

def connect_segments(
    current,
    following,
    line_spacing,
    safe_grid,
    clearance,
    metadata,
):
    """
    First attempt the natural coverage Dubins transition.

    If that intersects an obstacle:
        A* -> simplify -> Dubins smooth -> validate.
    """

    # Derived directly from line spacing.
    turn_radius = (
        line_spacing / 2.0
    )

    start_state = (
        current["end_ned"][0],
        current["end_ned"][1],
        current["heading"],
    )

    goal_state = (
        following["start_ned"][0],
        following["start_ned"][1],
        following["heading"],
    )

    # --------------------------------------------------------
    # Attempt direct coverage turn
    # --------------------------------------------------------

    direct, info = generate_dubins_path(
        start_state,
        goal_state,
        turning_radius=turn_radius,
        step=0.5,
    )

    validation = validate_path_on_safe_grid(
        direct,
        safe_grid,
        clearance,
        metadata,
    )

    if validation["safe"]:

        return (
            direct,
            "dubins",
        )

    # --------------------------------------------------------
    # A* around obstacle
    # --------------------------------------------------------

    astar = astar_grid(
        safe_grid,
        clearance,
        current["end_cell"],
        following["start_cell"],
        resolution=metadata[
            "resolution"
        ],
        preferred_clearance=max(
            12.0,
            line_spacing,
        ),
        clearance_weight=1.5,
    )

    simplified = simplify_grid_path(
        astar,
        safe_grid,
    )

    route_points = grid_path_to_ned(
        simplified,
        metadata,
    )

    smoothed, _ = (
        generate_dubins_via_waypoints(
            route_points,
            start_heading=current[
                "heading"
            ],
            goal_heading=following[
                "heading"
            ],
            turning_radius=turn_radius,
            step=0.5,
        )
    )

    validation = validate_path_on_safe_grid(
        smoothed,
        safe_grid,
        clearance,
        metadata,
    )

    if not validation["safe"]:

        raise RuntimeError(
            "A safe A* route exists between coverage "
            "segments, but the derived coverage turn "
            "cannot be Dubins-smoothed without collision."
        )

    return (
        smoothed,
        "astar+dubins",
    )




# ============================================================
# COMPLETE COVERAGE MISSION
# ============================================================

def build_coverage_mission(
    segments,
    line_spacing,
    safe_grid,
    clearance,
    metadata,
):
    """
    Build the complete continuous trajectory.
    """

    if not segments:
        raise RuntimeError(
            "No reachable coverage segments found"
        )

    path = []

    connector_types = []

    first_lane = sample_line(
        segments[0]["start_ned"],
        segments[0]["end_ned"],
        segments[0]["heading"],
    )

    append_samples(
        path,
        first_lane,
    )

    for index in range(
        len(segments) - 1
    ):

        current = segments[
            index
        ]

        following = segments[
            index + 1
        ]

        connector, connector_type = (
            connect_segments(
                current,
                following,
                line_spacing,
                safe_grid,
                clearance,
                metadata,
            )
        )

        append_samples(
            path,
            connector,
        )

        lane = sample_line(
            following["start_ned"],
            following["end_ned"],
            following["heading"],
        )

        append_samples(
            path,
            lane,
        )

        connector_types.append(
            connector_type
        )

    path = np.asarray(
        path,
        dtype=float,
    )

    validation = validate_path_on_safe_grid(
        path,
        safe_grid,
        clearance,
        metadata,
    )

    if not validation["safe"]:

        raise RuntimeError(
            "Final coverage mission failed "
            "collision validation"
        )

    return (
        path,
        connector_types,
        validation,
    )


# ============================================================
# PLOT RESULT
# ============================================================

def plot_result(
    raw_grid,
    safe_grid,
    metadata,
    first,
    second,
    segments,
    mission,
):

    display = display_map(
        raw_grid,
        safe_grid,
    )

    cmap = ListedColormap([
        "white",
        "orange",
        "black",
    ])

    plt.figure(
        figsize=(15, 9)
    )

    plt.imshow(
        display,
        origin="lower",
        extent=[
            metadata["east_min"],
            metadata["east_max"],
            metadata["north_min"],
            metadata["north_max"],
        ],
        interpolation="nearest",
        aspect="equal",
        cmap=cmap,
        vmin=0,
        vmax=2,
    )

    n_min = min(
        first[0],
        second[0],
    )

    n_max = max(
        first[0],
        second[0],
    )

    e_min = min(
        first[1],
        second[1],
    )

    e_max = max(
        first[1],
        second[1],
    )

    plt.plot(
        [
            e_min,
            e_max,
            e_max,
            e_min,
            e_min,
        ],
        [
            n_min,
            n_min,
            n_max,
            n_max,
            n_min,
        ],
        linestyle="--",
        linewidth=1.5,
        label="Requested coverage area",
    )

    for index, segment in enumerate(
        segments
    ):

        plt.plot(
            [
                segment["start_ned"][1],
                segment["end_ned"][1],
            ],
            [
                segment["start_ned"][0],
                segment["end_ned"][0],
            ],
            linewidth=1.0,
            alpha=0.5,
            label=(
                "Safe coverage lanes"
                if index == 0
                else None
            ),
        )

    plt.plot(
        mission[:, 1],
        mission[:, 0],
        linewidth=2.0,
        label="Final coverage mission",
    )

    plt.scatter(
        first[1],
        first[0],
        marker="x",
        s=90,
        label="Preferred start corner",
    )

    plt.xlabel("East [m]")
    plt.ylabel("North [m]")

    plt.title(
        "3D-Aware Map-Safe Coverage Mission"
    )

    plt.legend()
    plt.tight_layout()
    plt.show()


# ============================================================
# COMPLETE COVERAGE MISSION BUILDER
# ============================================================

def build_complete_mission(
    first,
    second,
    line_spacing,
    r1_start_ned,
    r1_start_heading,
    safe_grid,
    clearance,
    metadata,
    raw_grid=None,
):
    """
    Build the complete R1 mission:

        live R1 pose
        -> A* approach
        -> Dubins smoothing
        -> coverage mission

    Returns:
        complete_mission
        segments
        mission
        connector_types
        coverage_validation
        complete_validation
        approach_validation
        approach_grid
        lane_step
    """

    bounds = rectangle_grid_bounds(
        first,
        second,
        metadata,
        safe_grid.shape,
    )

    start_cell = nearest_safe_cell(
        safe_grid,
        metadata,
        first,
        bounds,
    )

    reachable = reachable_safe_water(
        safe_grid,
        start_cell,
    )

    segments, _, lane_step = (
        generate_coverage_segments(
            safe_grid,
            reachable,
            metadata,
            first,
            second,
            line_spacing,
        )
    )

    if not segments:
        raise RuntimeError(
            "No reachable coverage lanes were produced"
        )

    (
        approach,
        approach_grid,
        approach_validation,
    ) = build_approach_path(
        start_ned=r1_start_ned,
        start_heading=r1_start_heading,
        first_segment=segments[0],
        line_spacing=line_spacing,
        safe_grid=safe_grid,
        clearance=clearance,
        metadata=metadata,
        raw_grid=raw_grid,
    )

    (
        mission,
        connector_types,
        coverage_validation,
    ) = build_coverage_mission(
        segments,
        line_spacing,
        safe_grid,
        clearance,
        metadata,
    )

    complete_mission = np.vstack(
        (
            approach,
            mission[1:],
        )
    )

    # --------------------------------------------------------
    # Final mission validation
    #
    # If R1 spawned inside the additional orange planning
    # buffer, coverage_approach already validated the short
    # escape separately against:
    #
    #   - raw terrain
    #   - physical WAM-V footprint clearance
    #
    # Strict normal safe-grid validation therefore begins at
    # the first waypoint that has entered ordinary safe water.
    # --------------------------------------------------------

    safe_start_index = int(
        approach_validation.get(
            "safe_start_index",
            0,
        )
    )

    strict_complete_validation = (
        validate_path_on_safe_grid(
            complete_mission[
                safe_start_index:
            ],
            safe_grid,
            clearance,
            metadata,
        )
    )

    if not strict_complete_validation["safe"]:
        raise RuntimeError(
            "Combined R1 approach + coverage mission "
            "failed collision validation after "
            "entering normal safe water."
        )

    complete_validation = dict(
        strict_complete_validation
    )

    complete_validation[
        "escape_used"
    ] = bool(
        approach_validation.get(
            "escape_used",
            False,
        )
    )

    complete_validation[
        "safe_start_index"
    ] = safe_start_index

    # Report the true minimum terrain clearance across both
    # the permitted spawn escape and the strict safe mission.
    complete_validation[
        "minimum_clearance"
    ] = min(
        float(
            approach_validation[
                "minimum_clearance"
            ]
        ),
        float(
            strict_complete_validation[
                "minimum_clearance"
            ]
        ),
    )

    return (
        complete_mission,
        segments,
        mission,
        connector_types,
        coverage_validation,
        complete_validation,
        approach_validation,
        approach_grid,
        lane_step,
    )



# ============================================================
# MAIN INTERACTIVE DEMO
# ============================================================

def main():

    print()
    print(
        "Building 3D-aware Sydney navigation map..."
    )

    mesh = find_sydney_mesh()

    vertices = load_shore_vertices(
        mesh
    )

    triangles = load_shore_triangles(
        mesh
    )

    z_min, z_max = (
        wamv_navigation_vertical_band(
            vertical_safety_margin=0.5,
        )
    )

    raw_grid, metadata = (
        build_vertical_slab_occupancy_grid(
            vertices,
            triangles,
            z_min,
            z_max,
            resolution=2.0,
            margin=10.0,
        )
    )

    safe_grid, clearance, required = (
        build_safe_occupancy(
            raw_grid,
            metadata,
            safety_margin=1.0,
        )
    )

    print(
        "Map ready."
    )

    print()
    print(
        "White  = safe water"
    )

    print(
        "Orange = WAM-V safety buffer"
    )

    print(
        "Black  = collision-relevant terrain"
    )

    first, second = (
        select_coverage_rectangle(
            raw_grid,
            safe_grid,
            metadata,
        )
    )

    print()
    print(
        "Selected coverage rectangle."
    )

    while True:

        try:

            line_spacing = float(
                input(
                    "Enter coverage line spacing [m]: "
                )
            )

            if line_spacing <= 0.0:
                raise ValueError

            break

        except ValueError:

            print(
                "Line spacing must be a positive number."
            )

    bounds = rectangle_grid_bounds(
        first,
        second,
        metadata,
        safe_grid.shape,
    )

    (
        row_min,
        row_max,
        col_min,
        col_max,
    ) = bounds

    rectangle_cells = (
        safe_grid[
            row_min:
            row_max + 1,
            col_min:
            col_max + 1
        ]
    )

    unsafe_count = int(
        np.count_nonzero(
            rectangle_cells
        )
    )

    total_count = int(
        rectangle_cells.size
    )

    if unsafe_count > 0:

        print()
        print(
            "WARNING: requested coverage area "
            "contains unsafe terrain/buffer."
        )

        print(
            "The mission will cover reachable safe "
            "water and route around obstacles."
        )

        print(
            "Unsafe cells inside rectangle:",
            unsafe_count,
            "/",
            total_count,
        )

    print()
    print(
        "Waiting for live R1 state "
        "from /r1/vehicle_state..."
    )

    (
        r1_north,
        r1_east,
        r1_start_heading,
    ) = get_vehicle_state_once(
        topic="/r1/vehicle_state",
        timeout_sec=30.0,
    )

    r1_start_ned = (
        r1_north,
        r1_east,
    )

    print(
        "Live R1 start:"
    )
    print(
        "  North   :",
        round(r1_north, 3),
        "m",
    )
    print(
        "  East    :",
        round(r1_east, 3),
        "m",
    )
    print(
        "  Heading :",
        round(r1_start_heading, 4),
        "rad",
    )

    (
        complete_mission,
        segments,
        mission,
        connector_types,
        validation,
        complete_validation,
        approach_validation,
        approach_grid,
        lane_step,
    ) = build_complete_mission(
        first=first,
        second=second,
        line_spacing=line_spacing,
        r1_start_ned=r1_start_ned,
        r1_start_heading=r1_start_heading,
        safe_grid=safe_grid,
        clearance=clearance,
        metadata=metadata,
        raw_grid=raw_grid,
    )

    print()
    print("R1 -> coverage start:")
    print(
        "  A* waypoints      :",
        len(approach_grid),
    )
    print(
        "  Min clearance     :",
        round(
            approach_validation[
                "minimum_clearance"
            ],
            2,
        ),
        "m",
    )

    print()
    print("Complete mission:")
    print(
        "  Samples            :",
        len(complete_mission),
    )
    print(
        "  Path length        :",
        round(
            path_length(
                complete_mission
            ),
            2,
        ),
        "m",
    )
    print(
        "  Minimum clearance  :",
        round(
            complete_validation[
                "minimum_clearance"
            ],
            2,
        ),
        "m",
    )
    print(
        "  Collision check    : PASS"
    )

    unique_lanes = len(
        set(
            segment["lane_index"]
            for segment in segments
        )
    )

    split_counts = {}

    for segment in segments:

        lane = segment[
            "lane_index"
        ]

        split_counts[lane] = (
            split_counts.get(
                lane,
                0,
            )
            + 1
        )

    split_lanes = sum(
        1
        for count
        in split_counts.values()
        if count > 1
    )

    print()
    print("=" * 60)
    print("COVERAGE RESULT")
    print("=" * 60)

    print(
        "Requested line spacing:",
        line_spacing,
        "m",
    )

    print(
        "Actual grid lane step :",
        lane_step
        * metadata["resolution"],
        "m",
    )

    print(
        "Coverage lanes        :",
        unique_lanes,
    )

    print(
        "Safe lane segments    :",
        len(segments),
    )

    print(
        "Obstacle-split lanes  :",
        split_lanes,
    )

    print(
        "Direct turns          :",
        connector_types.count(
            "dubins"
        ),
    )

    print(
        "A* rerouted turns     :",
        connector_types.count(
            "astar+dubins"
        ),
    )

    print(
        "Final samples         :",
        len(mission),
    )

    print(
        "Final path length     :",
        round(
            path_length(
                mission
            ),
            2,
        ),
        "m",
    )

    print(
        "Minimum clearance     :",
        round(
            validation[
                "minimum_clearance"
            ],
            2,
        ),
        "m",
    )

    print(
        "Final collision check : PASS"
    )

    plot_result(
        raw_grid,
        safe_grid,
        metadata,
        first,
        second,
        segments,
        complete_mission,
    )


if __name__ == "__main__":
    main()
