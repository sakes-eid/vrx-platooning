#!/usr/bin/env python3

"""
Shared Sydney Regatta environment utilities.

This module is the authoritative environment layer for the platoon planner.

Coordinate convention used by this project:

    Gazebo world:
        X = East
        Y = North

    planner world_ned:
        x = North
        y = East

Therefore:

    north = gazebo_y
    east  = gazebo_x

All future occupancy, A*, Dubins, clearance, and coverage planning should
use the conversion functions in this module instead of implementing their
own coordinate transformations.
"""

from pathlib import Path


SYDNEY_MODEL_ROOT = (
    Path.home()
    / ".gz"
    / "fuel"
    / "fuel.gazebosim.org"
    / "openrobotics"
    / "models"
    / "sydney_regatta"
)

SYDNEY_MESH_NAME = "sydney_regatta_shore.dae"


def gazebo_to_ned(gazebo_x: float, gazebo_y: float):
    """
    Convert Sydney Gazebo planar coordinates to project world_ned.

    Gazebo:
        X = East
        Y = North

    world_ned:
        x = North
        y = East
    """
    north = float(gazebo_y)
    east = float(gazebo_x)

    return north, east


def ned_to_gazebo(north: float, east: float):
    """
    Convert project world_ned planar coordinates back to Gazebo world.
    """
    gazebo_x = float(east)
    gazebo_y = float(north)

    return gazebo_x, gazebo_y


def find_sydney_mesh():
    """
    Find the newest available Sydney Regatta shoreline COLLADA mesh.

    Gazebo Fuel may install the model under version directories such as:

        sydney_regatta/1/
        sydney_regatta/2/
        sydney_regatta/3/
    """

    if not SYDNEY_MODEL_ROOT.exists():
        raise FileNotFoundError(
            f"Sydney Regatta model directory not found: "
            f"{SYDNEY_MODEL_ROOT}"
        )

    candidates = list(
        SYDNEY_MODEL_ROOT.glob(
            f"*/meshes/{SYDNEY_MESH_NAME}"
        )
    )

    if not candidates:
        raise FileNotFoundError(
            f"Could not find {SYDNEY_MESH_NAME} under "
            f"{SYDNEY_MODEL_ROOT}"
        )

    def version_number(path: Path):
        try:
            return int(path.parents[1].name)
        except ValueError:
            return -1

    candidates.sort(
        key=version_number,
        reverse=True,
    )

    return candidates[0]


if __name__ == "__main__":
    mesh = find_sydney_mesh()

    print("Sydney environment module OK")
    print(f"Mesh: {mesh}")

    # Known R1 spawn from current bringup:
    # Gazebo approximately (-532, +162)
    north, east = gazebo_to_ned(
        -532.0,
        162.0,
    )

    print(
        "Example Gazebo (-532, 162) -> "
        f"world_ned ({north:.1f}, {east:.1f})"
    )


def load_shore_vertices(mesh_path=None):
    """
    Load Sydney terrain collision vertices from the COLLADA mesh.

    Returns:
        list of (north, east, up) tuples in project world_ned coordinates.
    """
    import xml.etree.ElementTree as ET

    if mesh_path is None:
        mesh_path = find_sydney_mesh()

    tree = ET.parse(mesh_path)
    root = tree.getroot()

    namespace = {
        "c": "http://www.collada.org/2005/11/COLLADASchema"
    }

    position_array = root.find(
        ".//c:float_array"
        "[@id='SydneyTerrainCollider-POSITION-array']",
        namespace,
    )

    if position_array is None:
        raise RuntimeError(
            "Could not find SydneyTerrainCollider POSITION array "
            f"in {mesh_path}"
        )

    raw_values = [
        float(value)
        for value in position_array.text.split()
    ]

    if len(raw_values) % 3 != 0:
        raise RuntimeError(
            "Terrain vertex array is not divisible into XYZ coordinates."
        )

    vertices = []

    for index in range(0, len(raw_values), 3):
        # Sydney mesh coordinates are stored in millimetres.
        gazebo_x = raw_values[index] * 0.001
        gazebo_y = raw_values[index + 1] * 0.001
        gazebo_z = raw_values[index + 2] * 0.001

        north, east = gazebo_to_ned(
            gazebo_x,
            gazebo_y,
        )

        vertices.append(
            (
                north,
                east,
                gazebo_z,
            )
        )

    return vertices


def terrain_bounds(vertices):
    """
    Return planar world_ned bounds for a terrain vertex collection.
    """
    if not vertices:
        raise ValueError("vertices must not be empty")

    north_values = [
        vertex[0]
        for vertex in vertices
    ]

    east_values = [
        vertex[1]
        for vertex in vertices
    ]

    return {
        "north_min": min(north_values),
        "north_max": max(north_values),
        "east_min": min(east_values),
        "east_max": max(east_values),
    }


def environment_summary():
    mesh = find_sydney_mesh()
    vertices = load_shore_vertices(mesh)
    bounds = terrain_bounds(vertices)

    print()
    print("Terrain extraction")
    print("------------------")
    print(f"Vertices : {len(vertices)}")
    print(
        f"North    : {bounds['north_min']:.2f} "
        f"to {bounds['north_max']:.2f} m"
    )
    print(
        f"East     : {bounds['east_min']:.2f} "
        f"to {bounds['east_max']:.2f} m"
    )


def load_shore_triangles(mesh_path=None):
    """
    Load terrain triangle connectivity from the Sydney COLLADA mesh.

    Returns:
        list of (vertex_index_a, vertex_index_b, vertex_index_c)

    Unlike the older implementation, the COLLADA input stride is derived
    from the file instead of being hard-coded.
    """
    import xml.etree.ElementTree as ET

    if mesh_path is None:
        mesh_path = find_sydney_mesh()

    tree = ET.parse(mesh_path)
    root = tree.getroot()

    namespace = {
        "c": "http://www.collada.org/2005/11/COLLADASchema"
    }

    triangles_element = root.find(
        ".//c:triangles",
        namespace,
    )

    if triangles_element is None:
        raise RuntimeError(
            f"No COLLADA triangle data found in {mesh_path}"
        )

    inputs = triangles_element.findall(
        "c:input",
        namespace,
    )

    if not inputs:
        raise RuntimeError(
            "Triangle element has no COLLADA input definitions."
        )

    offsets = [
        int(item.attrib.get("offset", "0"))
        for item in inputs
    ]

    stride = max(offsets) + 1

    vertex_input = None

    for item in inputs:
        if item.attrib.get("semantic") == "VERTEX":
            vertex_input = item
            break

    if vertex_input is None:
        raise RuntimeError(
            "Triangle element has no VERTEX input."
        )

    vertex_offset = int(
        vertex_input.attrib.get("offset", "0")
    )

    index_element = triangles_element.find(
        "c:p",
        namespace,
    )

    if index_element is None or not index_element.text:
        raise RuntimeError(
            "Triangle element has no index list."
        )

    raw_indices = [
        int(value)
        for value in index_element.text.split()
    ]

    if len(raw_indices) % stride != 0:
        raise RuntimeError(
            "COLLADA triangle index stream does not match "
            f"its declared stride ({stride})."
        )

    vertex_indices = raw_indices[
        vertex_offset::stride
    ]

    if len(vertex_indices) % 3 != 0:
        raise RuntimeError(
            "Terrain vertex indices cannot be grouped into triangles."
        )

    triangles = [
        (
            vertex_indices[index],
            vertex_indices[index + 1],
            vertex_indices[index + 2],
        )
        for index in range(
            0,
            len(vertex_indices),
            3,
        )
    ]

    declared_count = int(
        triangles_element.attrib.get(
            "count",
            len(triangles),
        )
    )

    if len(triangles) != declared_count:
        raise RuntimeError(
            "Terrain triangle count mismatch: "
            f"decoded={len(triangles)}, "
            f"declared={declared_count}"
        )

    return triangles


def build_occupancy_grid(
    vertices,
    triangles,
    resolution=2.0,
    margin=10.0,
):
    """
    Rasterize the Sydney terrain triangles into a world_ned occupancy grid.

    Grid convention:
        0 = free / navigable
        1 = occupied terrain

    Axis convention:
        rows    -> North
        columns -> East
    """
    import math
    import numpy as np

    if resolution <= 0.0:
        raise ValueError("resolution must be > 0")

    if margin < 0.0:
        raise ValueError("margin must be >= 0")

    bounds = terrain_bounds(vertices)

    north_min = (
        math.floor(
            (bounds["north_min"] - margin) / resolution
        )
        * resolution
    )

    north_max = (
        math.ceil(
            (bounds["north_max"] + margin) / resolution
        )
        * resolution
    )

    east_min = (
        math.floor(
            (bounds["east_min"] - margin) / resolution
        )
        * resolution
    )

    east_max = (
        math.ceil(
            (bounds["east_max"] + margin) / resolution
        )
        * resolution
    )

    height = int(
        math.ceil(
            (north_max - north_min) / resolution
        )
    )

    width = int(
        math.ceil(
            (east_max - east_min) / resolution
        )
    )

    occupancy = np.zeros(
        (height, width),
        dtype=np.uint8,
    )

    vertex_array = np.asarray(
        vertices,
        dtype=np.float64,
    )

    north_centers = (
        north_min
        + (
            np.arange(height)
            + 0.5
        )
        * resolution
    )

    east_centers = (
        east_min
        + (
            np.arange(width)
            + 0.5
        )
        * resolution
    )

    rasterized_triangles = 0

    for triangle in triangles:
        tri = vertex_array[
            list(triangle),
            :2,
        ]

        n0 = float(np.min(tri[:, 0]))
        n1 = float(np.max(tri[:, 0]))
        e0 = float(np.min(tri[:, 1]))
        e1 = float(np.max(tri[:, 1]))

        row0 = max(
            0,
            int(
                math.floor(
                    (n0 - north_min)
                    / resolution
                )
            ),
        )

        row1 = min(
            height - 1,
            int(
                math.floor(
                    (n1 - north_min)
                    / resolution
                )
            ),
        )

        col0 = max(
            0,
            int(
                math.floor(
                    (e0 - east_min)
                    / resolution
                )
            ),
        )

        col1 = min(
            width - 1,
            int(
                math.floor(
                    (e1 - east_min)
                    / resolution
                )
            ),
        )

        if row1 < row0 or col1 < col0:
            continue

        ns = north_centers[
            row0:row1 + 1
        ]

        es = east_centers[
            col0:col1 + 1
        ]

        nn, ee = np.meshgrid(
            ns,
            es,
            indexing="ij",
        )

        # Vectorized point-in-triangle test using barycentric signs.
        a = tri[0]
        b = tri[1]
        c = tri[2]

        def edge(px, py, p1, p2):
            return (
                (px - p2[0])
                * (p1[1] - p2[1])
                -
                (p1[0] - p2[0])
                * (py - p2[1])
            )

        d1 = edge(
            nn,
            ee,
            a,
            b,
        )

        d2 = edge(
            nn,
            ee,
            b,
            c,
        )

        d3 = edge(
            nn,
            ee,
            c,
            a,
        )

        eps = 1e-9

        has_negative = (
            (d1 < -eps)
            | (d2 < -eps)
            | (d3 < -eps)
        )

        has_positive = (
            (d1 > eps)
            | (d2 > eps)
            | (d3 > eps)
        )

        inside = ~(
            has_negative
            & has_positive
        )

        if np.any(inside):
            occupancy[
                row0:row1 + 1,
                col0:col1 + 1,
            ][inside] = 1

        rasterized_triangles += 1

    metadata = {
        "frame_id": "world_ned",
        "resolution": float(resolution),

        "north_min": float(north_min),
        "north_max": float(north_max),

        "east_min": float(east_min),
        "east_max": float(east_max),

        "height": int(height),
        "width": int(width),

        "rasterized_triangles":
            int(rasterized_triangles),
    }

    return occupancy, metadata


def ned_to_grid(
    north,
    east,
    metadata,
):
    """
    Convert world_ned coordinates to occupancy-grid row/column.
    """
    import math

    row = int(
        math.floor(
            (
                float(north)
                - metadata["north_min"]
            )
            / metadata["resolution"]
        )
    )

    column = int(
        math.floor(
            (
                float(east)
                - metadata["east_min"]
            )
            / metadata["resolution"]
        )
    )

    return row, column


def grid_to_ned(
    row,
    column,
    metadata,
):
    """
    Return the world_ned coordinate of a grid-cell centre.
    """
    north = (
        metadata["north_min"]
        + (
            int(row)
            + 0.5
        )
        * metadata["resolution"]
    )

    east = (
        metadata["east_min"]
        + (
            int(column)
            + 0.5
        )
        * metadata["resolution"]
    )

    return north, east


def occupancy_at(
    occupancy,
    metadata,
    north,
    east,
):
    """
    Query occupancy at a world_ned coordinate.

    Returns:
        0     free
        1     occupied
        None  outside map
    """
    row, column = ned_to_grid(
        north,
        east,
        metadata,
    )

    if not (
        0 <= row < occupancy.shape[0]
        and
        0 <= column < occupancy.shape[1]
    ):
        return None

    return int(
        occupancy[
            row,
            column,
        ]
    )


# ============================================================
# WAM-V GEOMETRY / NAVIGATION CLEARANCE
# ============================================================

# Keep these values consistent with
# proactive_follower_controller_v22.py.
WAMV_GPS_X_FROM_BASE = -0.85
WAMV_FRONT_EXTENT = 2.549
WAMV_REAR_EXTENT = 2.822
WAMV_HALF_WIDTH = 1.267

DEFAULT_SHORE_SAFETY_MARGIN = 1.0


def wamv_gps_footprint_radius():
    """
    Conservative orientation-independent WAM-V radius measured from
    the GPS reference point used by VehicleState.

    A circular bound is intentionally used for global planning so the
    safe map is valid regardless of vessel heading.
    """
    import math

    # GPS = base + [-0.85, 0] in the vessel frame.
    #
    # Relative to GPS:
    #
    # front corner longitudinal distance:
    #     FRONT_EXTENT - GPS_X_FROM_BASE
    #
    # rear corner longitudinal distance:
    #     -REAR_EXTENT - GPS_X_FROM_BASE
    front_x = (
        WAMV_FRONT_EXTENT
        - WAMV_GPS_X_FROM_BASE
    )

    rear_x = (
        -WAMV_REAR_EXTENT
        - WAMV_GPS_X_FROM_BASE
    )

    front_radius = math.hypot(
        front_x,
        WAMV_HALF_WIDTH,
    )

    rear_radius = math.hypot(
        rear_x,
        WAMV_HALF_WIDTH,
    )

    return max(
        front_radius,
        rear_radius,
    )


def build_clearance_map(
    occupancy,
    metadata,
):
    """
    Calculate conservative distance from each grid-cell centre to
    occupied terrain.

    Returns clearance in metres.

    scipy.ndimage.distance_transform_edt gives centre-to-centre
    Euclidean distance to the nearest occupied cell.

    We subtract half a grid-cell diagonal so the reported distance is
    conservative relative to the physical area represented by that
    occupied cell.
    """
    import math
    import numpy as np

    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError as exc:
        raise RuntimeError(
            "SciPy is required for clearance-map generation. "
            "Install python3-scipy."
        ) from exc

    resolution = float(
        metadata["resolution"]
    )

    raw_clearance = distance_transform_edt(
        occupancy == 0,
        sampling=(
            resolution,
            resolution,
        ),
    )

    half_cell_diagonal = (
        0.5
        * math.sqrt(2.0)
        * resolution
    )

    clearance = np.maximum(
        raw_clearance
        - half_cell_diagonal,
        0.0,
    )

    # Terrain itself always has exactly zero clearance.
    clearance[
        occupancy != 0
    ] = 0.0

    return clearance


def build_safe_occupancy(
    occupancy,
    metadata,
    safety_margin=DEFAULT_SHORE_SAFETY_MARGIN,
):
    """
    Inflate shoreline obstacles using an orientation-independent
    WAM-V footprint plus an additional safety margin.

    Returns:
        safe_occupancy:
            0 = navigable
            1 = unsafe

        clearance:
            conservative terrain clearance in metres

        required_clearance:
            WAM-V radius + requested safety margin
    """
    import numpy as np

    if safety_margin < 0.0:
        raise ValueError(
            "safety_margin must be >= 0"
        )

    clearance = build_clearance_map(
        occupancy,
        metadata,
    )

    required_clearance = (
        wamv_gps_footprint_radius()
        + float(safety_margin)
    )

    safe_occupancy = np.where(
        clearance >= required_clearance,
        0,
        1,
    ).astype(np.uint8)

    return (
        safe_occupancy,
        clearance,
        required_clearance,
    )


def clearance_at(
    clearance,
    metadata,
    north,
    east,
):
    """
    Return conservative terrain clearance at a world_ned point.

    Returns None outside the grid.
    """
    row, column = ned_to_grid(
        north,
        east,
        metadata,
    )

    if not (
        0 <= row < clearance.shape[0]
        and
        0 <= column < clearance.shape[1]
    ):
        return None

    return float(
        clearance[
            row,
            column,
        ]
    )
