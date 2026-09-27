import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from pathlib import Path as FilePath
import os


# ============================================================
# PROJECT PATHS
# ============================================================

# Directory containing this Python file
SCRIPT_DIR = FilePath(__file__).resolve().parent

# Project folders
DATA_DIR = SCRIPT_DIR / "data"
OUTPUT_DIR = SCRIPT_DIR / "output"

# Create folders automatically if they do not exist
DATA_DIR.mkdir(parents=True, exist_ok=True)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# CONFIG
# ============================================================

# Sydney Regatta map area
x_min = -700.0
x_max = -300.0

y_min = 100.0
y_max = 400.0

resolution = 2.0


# Boat initial position
boat_x = -533.7402476486775
boat_y = 161.7656784075805


# ============================================================
# FIND SYDNEY REGATTA DAE FILE
# ============================================================

home_dir = FilePath.home()

fuel_model_dir = (
    home_dir
    / ".gz"
    / "fuel"
    / "fuel.gazebosim.org"
    / "openrobotics"
    / "models"
    / "sydney_regatta"
)


def find_dae_file():
    """
    Find the Sydney Regatta shore DAE file automatically.

    Gazebo Fuel may store different model versions
    in folders such as:
        1/
        2/
        3/
        ...

    Therefore we search all versions instead of
    hard-coding version 3.
    """

    if not fuel_model_dir.exists():
        raise FileNotFoundError(
            "\nSydney Regatta model directory was not found:\n"
            f"{fuel_model_dir}\n\n"
            "Please make sure the Sydney Regatta Gazebo model "
            "has been downloaded."
        )

    candidates = list(
        fuel_model_dir.glob(
            "*/meshes/sydney_regatta_shore.dae"
        )
    )

    if len(candidates) == 0:
        raise FileNotFoundError(
            "\nCould not find:\n"
            "sydney_regatta_shore.dae\n\n"
            f"inside:\n{fuel_model_dir}"
        )

    # Sort by model version if possible
    def version_number(path):
        try:
            return int(path.parents[1].name)
        except ValueError:
            return -1

    candidates.sort(
        key=version_number,
        reverse=True
    )

    return candidates[0]


dae_path = find_dae_file()


# Output files
png_path = (
    OUTPUT_DIR
    / "sydney_local_occupancy.png"
)

npy_path = (
    DATA_DIR
    / "sydney_local_occupancy.npy"
)


print()
print("========================================")
print("FILES")
print("========================================")

print("DAE:")
print(dae_path)

print()
print("Occupancy grid output:")
print(npy_path)

print()
print("Image output:")
print(png_path)


# ============================================================
# READ DAE
# ============================================================

tree = ET.parse(
    dae_path
)

root = tree.getroot()


ns = {
    "c":
    "http://www.collada.org/2005/11/COLLADASchema"
}


# ============================================================
# LOAD TERRAIN VERTICES
# ============================================================

arr = root.find(
    ".//c:float_array"
    "[@id='SydneyTerrainCollider-POSITION-array']",
    ns
)


if arr is None:

    raise RuntimeError(
        "Could not find "
        "SydneyTerrainCollider POSITION array"
    )


values = np.array(
    list(
        map(
            float,
            arr.text.split()
        )
    ),
    dtype=np.float64
)


vertices = values.reshape(
    -1,
    3
)


# Sydney Regatta DAE node scale
vertices *= 0.001


print()
print("========================================")
print("TERRAIN")
print("========================================")

print(
    "Vertices:",
    len(vertices)
)

print(
    "X:",
    vertices[:, 0].min(),
    "to",
    vertices[:, 0].max()
)

print(
    "Y:",
    vertices[:, 1].min(),
    "to",
    vertices[:, 1].max()
)

print(
    "Z:",
    vertices[:, 2].min(),
    "to",
    vertices[:, 2].max()
)


# ============================================================
# LOAD TRIANGLE INDICES
# ============================================================

triangles_element = root.find(
    ".//c:triangles",
    ns
)


if triangles_element is None:

    raise RuntimeError(
        "Could not find triangle element"
    )


triangle_count_expected = int(
    triangles_element.attrib[
        "count"
    ]
)


p = triangles_element.find(
    "c:p",
    ns
)


if p is None:

    raise RuntimeError(
        "Could not find triangle index list"
    )


indices_raw = np.array(
    list(
        map(
            int,
            p.text.split()
        )
    ),
    dtype=np.int64
)


# COLLADA triangle input:
#
# VERTEX   offset = 0
# NORMAL   offset = 1
# TEXCOORD offset = 2
#
# Therefore each vertex contains:
#
# [vertex_index,
#  normal_index,
#  texcoord_index]

stride = 3


vertex_indices = (
    indices_raw[
        0::stride
    ]
)


if (
    len(vertex_indices) % 3
    != 0
):

    raise RuntimeError(
        "Triangle vertex index count "
        "is invalid"
    )


triangles = (
    vertex_indices
    .reshape(
        -1,
        3
    )
)


print()
print(
    "Triangles:",
    len(triangles)
)

print(
    "Expected triangles:",
    triangle_count_expected
)


# ============================================================
# CREATE GRID
# ============================================================

grid_width = int(
    np.ceil(
        (x_max - x_min)
        / resolution
    )
)


grid_height = int(
    np.ceil(
        (y_max - y_min)
        / resolution
    )
)


grid = np.zeros(
    (
        grid_height,
        grid_width
    ),
    dtype=np.uint8
)


print()
print("========================================")
print("GRID")
print("========================================")

print(
    "Grid:",
    grid_width,
    "x",
    grid_height
)

print(
    "Resolution:",
    resolution,
    "m"
)


# ============================================================
# CELL CENTER COORDINATES
# ============================================================

x_centers = (
    x_min
    +
    (
        np.arange(
            grid_width
        )
        + 0.5
    )
    * resolution
)


y_centers = (
    y_min
    +
    (
        np.arange(
            grid_height
        )
        + 0.5
    )
    * resolution
)


# ============================================================
# RASTERIZE TERRAIN TRIANGLES
# ============================================================

triangles_in_roi = 0


for tri_indices in triangles:

    tri_xyz = vertices[
        tri_indices
    ]


    # Only XY projection is needed
    tri_xy = tri_xyz[
        :,
        :2
    ]


    tx_min = (
        tri_xy[:, 0]
        .min()
    )

    tx_max = (
        tri_xy[:, 0]
        .max()
    )

    ty_min = (
        tri_xy[:, 1]
        .min()
    )

    ty_max = (
        tri_xy[:, 1]
        .max()
    )


    # Skip triangles outside ROI

    if tx_max < x_min:
        continue

    if tx_min > x_max:
        continue

    if ty_max < y_min:
        continue

    if ty_min > y_max:
        continue


    triangles_in_roi += 1


    # --------------------------------------------------------
    # Triangle bounding box -> grid cells
    # --------------------------------------------------------

    ix0 = max(
        0,
        int(
            np.floor(
                (
                    tx_min
                    - x_min
                )
                / resolution
            )
        )
    )


    ix1 = min(
        grid_width - 1,
        int(
            np.floor(
                (
                    tx_max
                    - x_min
                )
                / resolution
            )
        )
    )


    iy0 = max(
        0,
        int(
            np.floor(
                (
                    ty_min
                    - y_min
                )
                / resolution
            )
        )
    )


    iy1 = min(
        grid_height - 1,
        int(
            np.floor(
                (
                    ty_max
                    - y_min
                )
                / resolution
            )
        )
    )


    if (
        ix1 < ix0
        or
        iy1 < iy0
    ):

        continue


    # --------------------------------------------------------
    # Grid-cell centers inside bounding box
    # --------------------------------------------------------

    xs = x_centers[
        ix0:
        ix1 + 1
    ]


    ys = y_centers[
        iy0:
        iy1 + 1
    ]


    xx, yy = np.meshgrid(
        xs,
        ys
    )


    pts = np.column_stack(
        (
            xx.ravel(),
            yy.ravel()
        )
    )


    triangle_path = Path(
        tri_xy
    )


    inside = (
        triangle_path
        .contains_points(
            pts,
            radius=1e-9
        )
    )


    inside = inside.reshape(
        len(ys),
        len(xs)
    )


    subgrid = grid[
        iy0:
        iy1 + 1,

        ix0:
        ix1 + 1
    ]


    subgrid[
        inside
    ] = 1


# ============================================================
# BOAT GRID POSITION
# ============================================================

boat_ix = int(
    np.floor(
        (
            boat_x
            - x_min
        )
        / resolution
    )
)


boat_iy = int(
    np.floor(
        (
            boat_y
            - y_min
        )
        / resolution
    )
)


print()
print("========================================")
print("RESULT")
print("========================================")

print(
    "Triangles intersecting ROI:",
    triangles_in_roi
)


occupied = int(
    np.sum(
        grid
    )
)


free = int(
    grid.size
    - occupied
)


print(
    "Occupied cells:",
    occupied
)

print(
    "Free cells:",
    free
)


print()
print("Boat:")

print(
    "x =",
    boat_x
)

print(
    "y =",
    boat_y
)

print(
    "grid ix =",
    boat_ix
)

print(
    "grid iy =",
    boat_iy
)


if (
    0 <= boat_ix < grid_width
    and
    0 <= boat_iy < grid_height
):

    print(
        "Boat cell occupancy =",
        int(
            grid[
                boat_iy,
                boat_ix
            ]
        )
    )

else:

    print(
        "WARNING: "
        "Boat is outside grid"
    )


# ============================================================
# SAVE OCCUPANCY GRID
# ============================================================

np.save(
    npy_path,
    grid
)


# ============================================================
# PLOT
# ============================================================

plt.figure(
    figsize=(
        12,
        8
    )
)


plt.imshow(
    grid,
    origin="lower",

    extent=[
        x_min,
        x_max,
        y_min,
        y_max
    ],

    cmap="binary",

    interpolation="nearest",

    vmin=0,
    vmax=1,

    aspect="equal"
)


plt.scatter(
    boat_x,
    boat_y,

    marker="x",

    s=120,

    linewidths=2.5,

    label="Boat"
)


plt.xlabel(
    "X [m]"
)


plt.ylabel(
    "Y [m]"
)


plt.title(
    "Sydney Regatta Local "
    "Occupancy Grid"
)


plt.xlim(
    x_min,
    x_max
)


plt.ylim(
    y_min,
    y_max
)


plt.grid(
    True,
    alpha=0.35
)


plt.legend()


plt.tight_layout()


plt.savefig(
    png_path,
    dpi=200
)


print()
print("========================================")
print("SAVED")
print("========================================")

print(
    "Occupancy grid:"
)

print(
    npy_path
)

print()

print(
    "Figure:"
)

print(
    png_path
)


plt.show()