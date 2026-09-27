import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import csv

from pathlib import Path


# ============================================================
# PROJECT PATHS
# ============================================================

# Directory containing this Python file
BASE_DIR = Path(__file__).resolve().parent

DATA_DIR = BASE_DIR / "data"
OUTPUT_DIR = BASE_DIR / "output"

# Automatically create output directory
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Input occupancy grid
occupancy_path = DATA_DIR / "sydney_local_occupancy.npy"

# Output files
path_npy = OUTPUT_DIR / "sydney_coverage_path.npy"
path_csv = OUTPUT_DIR / "sydney_coverage_path.csv"
path_png = OUTPUT_DIR / "sydney_coverage_path.png"


# ============================================================
# CONFIG
# ============================================================

# Occupancy map coordinate range
# Must match sydney_occupancy.py

map_x_min = -700.0
map_x_max = -300.0

map_y_min = 100.0
map_y_max = 400.0

resolution = 2.0


# Boat INITIAL position

boat_start_x = -533.7402476486775
boat_start_y = 161.7656784075805


# ============================================================
# CHECK INPUT FILE
# ============================================================

if not occupancy_path.exists():

    raise FileNotFoundError(
        "\nOccupancy map was not found:\n"
        f"{occupancy_path}\n\n"
        "Expected project structure:\n"
        "ky_py_pkg/\n"
        "├── sydney_coverage.py\n"
        "└── data/\n"
        "    └── sydney_local_occupancy.npy\n"
    )


# ============================================================
# LOAD OCCUPANCY GRID
# ============================================================

occupancy = np.load(occupancy_path)

grid_height, grid_width = occupancy.shape

print()
print("========================================")
print("MAP")
print("========================================")

print("Occupancy file:", occupancy_path)
print("Grid shape:", occupancy.shape)
print("Resolution:", resolution, "m")
print("X:", map_x_min, "to", map_x_max)
print("Y:", map_y_min, "to", map_y_max)


# ============================================================
# COORDINATE CONVERSION
# ============================================================

def world_to_grid(x, y):

    col = int(
        np.floor(
            (x - map_x_min) / resolution
        )
    )

    row = int(
        np.floor(
            (y - map_y_min) / resolution
        )
    )

    return row, col


def grid_to_world(row, col):

    x = (
        map_x_min
        + (col + 0.5) * resolution
    )

    y = (
        map_y_min
        + (row + 0.5) * resolution
    )

    return x, y


def inside_grid(row, col):

    return (
        0 <= row < grid_height
        and
        0 <= col < grid_width
    )


def is_free(row, col):

    if not inside_grid(row, col):
        return False

    return occupancy[row, col] == 0


# ============================================================
# CHECK BOAT START
# ============================================================

boat_start_grid = world_to_grid(
    boat_start_x,
    boat_start_y
)

print()
print("========================================")
print("BOAT START")
print("========================================")

print(
    "World:",
    boat_start_x,
    boat_start_y
)

print(
    "Grid:",
    boat_start_grid
)

if not is_free(*boat_start_grid):

    raise RuntimeError(
        "Boat starting position is occupied."
    )


# ============================================================
# A*
# ============================================================

def heuristic(a, b):

    return math.hypot(
        a[0] - b[0],
        a[1] - b[1]
    )


def astar(start, goal):

    if not is_free(*start):

        print(
            "WARNING: A* start occupied:",
            start
        )

        return None


    if not is_free(*goal):

        print(
            "WARNING: A* goal occupied:",
            goal
        )

        return None


    neighbors = [

        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),

        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2))
    ]


    open_heap = []

    heapq.heappush(
        open_heap,
        (
            0.0,
            start
        )
    )


    came_from = {}

    g_score = {
        start: 0.0
    }

    closed = set()


    while open_heap:

        _, current = heapq.heappop(
            open_heap
        )

        if current in closed:
            continue


        if current == goal:

            path = [current]

            while current in came_from:

                current = came_from[
                    current
                ]

                path.append(current)

            path.reverse()

            return path


        closed.add(current)

        cr, cc = current


        for dr, dc, move_cost in neighbors:

            nr = cr + dr
            nc = cc + dc

            neighbor = (
                nr,
                nc
            )


            if not is_free(
                nr,
                nc
            ):
                continue


            # Prevent diagonal corner cutting

            if dr != 0 and dc != 0:

                if not (
                    is_free(
                        cr + dr,
                        cc
                    )
                    and
                    is_free(
                        cr,
                        cc + dc
                    )
                ):
                    continue


            tentative_g = (
                g_score[current]
                + move_cost
            )


            if tentative_g < g_score.get(
                neighbor,
                float("inf")
            ):

                came_from[
                    neighbor
                ] = current

                g_score[
                    neighbor
                ] = tentative_g


                f_score = (
                    tentative_g
                    + heuristic(
                        neighbor,
                        goal
                    )
                )


                heapq.heappush(
                    open_heap,
                    (
                        f_score,
                        neighbor
                    )
                )


    return None


# ============================================================
# INTERACTIVE RECTANGLE SELECTION
# ============================================================

def select_coverage_rectangle():

    print()
    print("========================================")
    print("SELECT COVERAGE AREA")
    print("========================================")
    print()

    print(
        "Click TWO opposite corners "
        "of the coverage rectangle."
    )


    fig, ax = plt.subplots(
        figsize=(12, 8)
    )


    ax.imshow(
        occupancy,
        origin="lower",
        extent=[
            map_x_min,
            map_x_max,
            map_y_min,
            map_y_max
        ],
        cmap="binary",
        interpolation="nearest",
        vmin=0,
        vmax=1,
        aspect="equal"
    )


    ax.scatter(
        boat_start_x,
        boat_start_y,
        marker="x",
        s=140,
        linewidths=3,
        label="Boat start"
    )


    ax.set_title(
        "Click TWO opposite corners "
        "of the coverage rectangle"
    )

    ax.set_xlabel(
        "X [m]"
    )

    ax.set_ylabel(
        "Y [m]"
    )

    ax.grid(
        True,
        alpha=0.3
    )

    ax.legend()


    points = plt.ginput(
        2,
        timeout=-1
    )


    plt.close(
        fig
    )


    if len(points) != 2:

        raise RuntimeError(
            "Two points were not selected."
        )


    x1, y1 = points[0]
    x2, y2 = points[1]


    rect_x_min = max(
        min(x1, x2),
        map_x_min
    )

    rect_x_max = min(
        max(x1, x2),
        map_x_max
    )

    rect_y_min = max(
        min(y1, y2),
        map_y_min
    )

    rect_y_max = min(
        max(y1, y2),
        map_y_max
    )


    print()
    print("Selected rectangle:")

    print(
        "X:",
        round(rect_x_min, 2),
        "to",
        round(rect_x_max, 2)
    )

    print(
        "Y:",
        round(rect_y_min, 2),
        "to",
        round(rect_y_max, 2)
    )


    return (
        rect_x_min,
        rect_x_max,
        rect_y_min,
        rect_y_max
    )


# ============================================================
# SELECT RECTANGLE
# ============================================================

(
    coverage_x_min,
    coverage_x_max,
    coverage_y_min,
    coverage_y_max
) = select_coverage_rectangle()


# ============================================================
# PATH WIDTH
# ============================================================

print()
print("========================================")
print("PATH WIDTH")
print("========================================")


try:

    path_width = float(
        input(
            "Enter coverage path width [m] "
            "(example 10): "
        )
    )

except ValueError:

    raise RuntimeError(
        "Path width must be a number."
    )


if path_width <= 0:

    raise RuntimeError(
        "Path width must be > 0."
    )


row_spacing = max(
    1,
    int(
        round(
            path_width / resolution
        )
    )
)


print()
print(
    "Path width:",
    path_width,
    "m"
)

print(
    "Grid row spacing:",
    row_spacing
)


# ============================================================
# RECTANGLE -> GRID
# ============================================================

row_min, col_min = world_to_grid(
    coverage_x_min,
    coverage_y_min
)

row_max, col_max = world_to_grid(
    coverage_x_max,
    coverage_y_max
)


row_min = max(
    0,
    min(
        row_min,
        grid_height - 1
    )
)

row_max = max(
    0,
    min(
        row_max,
        grid_height - 1
    )
)

col_min = max(
    0,
    min(
        col_min,
        grid_width - 1
    )
)

col_max = max(
    0,
    min(
        col_max,
        grid_width - 1
    )
)


if row_min > row_max:

    row_min, row_max = (
        row_max,
        row_min
    )


if col_min > col_max:

    col_min, col_max = (
        col_max,
        col_min
    )


# ============================================================
# FIND FREE SEGMENTS
# ============================================================

def find_free_segments(
    row,
    col_start,
    col_end
):

    segments = []

    segment_start = None


    for col in range(
        col_start,
        col_end + 1
    ):

        free = is_free(
            row,
            col
        )


        if free:

            if segment_start is None:

                segment_start = col


        else:

            if segment_start is not None:

                segments.append(
                    (
                        segment_start,
                        col - 1
                    )
                )

                segment_start = None


    if segment_start is not None:

        segments.append(
            (
                segment_start,
                col_end
            )
        )


    return segments


# ============================================================
# BUILD COVERAGE TARGETS
# ============================================================

coverage_targets = []

direction_left_to_right = True


rows = list(
    range(
        row_min,
        row_max + 1,
        row_spacing
    )
)


for row in rows:

    segments = find_free_segments(
        row,
        col_min,
        col_max
    )


    if len(segments) == 0:

        continue


    if direction_left_to_right:

        ordered_segments = segments

    else:

        ordered_segments = list(
            reversed(
                segments
            )
        )


    for seg_start, seg_end in ordered_segments:

        if direction_left_to_right:

            start_point = (
                row,
                seg_start
            )

            end_point = (
                row,
                seg_end
            )

        else:

            start_point = (
                row,
                seg_end
            )

            end_point = (
                row,
                seg_start
            )


        coverage_targets.append(
            start_point
        )


        if end_point != start_point:

            coverage_targets.append(
                end_point
            )


    direction_left_to_right = (
        not direction_left_to_right
    )


if len(coverage_targets) == 0:

    raise RuntimeError(
        "No free coverage path "
        "inside selected rectangle."
    )


print()
print("========================================")
print("COVERAGE")
print("========================================")

print(
    "Number of coverage targets:",
    len(coverage_targets)
)


# ============================================================
# CONNECT COVERAGE TARGETS WITH A*
# ============================================================

full_grid_path = []

current_position = boat_start_grid


print()
print(
    "Planning path from boat "
    "to coverage area..."
)


for target in coverage_targets:

    segment = astar(
        current_position,
        target
    )


    if segment is None:

        print(
            "WARNING: Could not reach target:",
            target
        )

        continue


    if len(full_grid_path) == 0:

        full_grid_path.extend(
            segment
        )

    else:

        full_grid_path.extend(
            segment[1:]
        )


    current_position = target


# ============================================================
# RETURN TO BOAT START
# ============================================================

print()
print(
    "Planning return path "
    "to initial boat position..."
)


return_path = astar(
    current_position,
    boat_start_grid
)


if return_path is None:

    print(
        "WARNING: Could not find "
        "return path to boat start."
    )

else:

    full_grid_path.extend(
        return_path[1:]
    )


# ============================================================
# GRID PATH -> WORLD PATH
# ============================================================

world_path = []


for row, col in full_grid_path:

    x, y = grid_to_world(
        row,
        col
    )

    world_path.append(
        (
            x,
            y
        )
    )


world_path = np.asarray(
    world_path,
    dtype=np.float64
)


if len(world_path) == 0:

    raise RuntimeError(
        "No valid path was generated."
    )


# ============================================================
# SAVE NPY
# ============================================================

np.save(
    path_npy,
    world_path
)


# ============================================================
# SAVE CSV
# ============================================================

with open(
    path_csv,
    "w",
    newline=""
) as f:

    writer = csv.writer(
        f
    )


    writer.writerow([
        "waypoint_id",
        "x",
        "y"
    ])


    for i, point in enumerate(
        world_path
    ):

        writer.writerow([
            i,
            point[0],
            point[1]
        ])


print()
print("========================================")
print("PATH OUTPUT")
print("========================================")

print(
    "NPY:",
    path_npy
)

print(
    "CSV:",
    path_csv
)

print(
    "Number of waypoints:",
    len(world_path)
)

print(
    "First waypoint:",
    world_path[0]
)

print(
    "Last waypoint:",
    world_path[-1]
)


# ============================================================
# CALCULATE PATH LENGTH
# ============================================================

if len(world_path) > 1:

    dx = np.diff(
        world_path[:, 0]
    )

    dy = np.diff(
        world_path[:, 1]
    )

    total_distance = np.sum(
        np.sqrt(
            dx**2
            +
            dy**2
        )
    )

else:

    total_distance = 0.0


print()
print(
    "Total path distance:",
    round(
        total_distance,
        2
    ),
    "m"
)


# ============================================================
# PLOT RESULT
# ============================================================

fig, ax = plt.subplots(
    figsize=(12, 8)
)


ax.imshow(
    occupancy,
    origin="lower",
    extent=[
        map_x_min,
        map_x_max,
        map_y_min,
        map_y_max
    ],
    cmap="binary",
    interpolation="nearest",
    vmin=0,
    vmax=1,
    aspect="equal"
)


# ============================================================
# COVERAGE RECTANGLE
# ============================================================

rectangle_x = [
    coverage_x_min,
    coverage_x_max,
    coverage_x_max,
    coverage_x_min,
    coverage_x_min
]

rectangle_y = [
    coverage_y_min,
    coverage_y_min,
    coverage_y_max,
    coverage_y_max,
    coverage_y_min
]


ax.plot(
    rectangle_x,
    rectangle_y,
    linestyle="--",
    linewidth=2,
    label="Coverage area"
)


# ============================================================
# PATH
# ============================================================

ax.plot(
    world_path[:, 0],
    world_path[:, 1],
    linewidth=2,
    label="Coverage path"
)


# ============================================================
# BOAT START
# ============================================================

ax.scatter(
    boat_start_x,
    boat_start_y,
    marker="x",
    s=160,
    linewidths=3,
    label="Boat start / return"
)


# ============================================================
# COVERAGE ENTRY
# ============================================================

first_target_world = grid_to_world(
    *coverage_targets[0]
)


ax.scatter(
    first_target_world[0],
    first_target_world[1],
    marker="o",
    s=80,
    label="Coverage entry"
)


# ============================================================
# PLOT SETTINGS
# ============================================================

ax.set_xlabel(
    "X [m]"
)

ax.set_ylabel(
    "Y [m]"
)

ax.set_title(
    "Sydney Regatta Coverage Path"
)

ax.set_xlim(
    map_x_min,
    map_x_max
)

ax.set_ylim(
    map_y_min,
    map_y_max
)

ax.grid(
    True,
    alpha=0.3
)

ax.legend()

plt.tight_layout()


plt.savefig(
    path_png,
    dpi=200
)


print(
    "PNG:",
    path_png
)


plt.show()