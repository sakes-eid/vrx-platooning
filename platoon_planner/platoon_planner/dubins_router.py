#!/usr/bin/env python3

"""
Forward-only Dubins routing in the project's world_ned frame.

Coordinates:
    x = North
    y = East

Heading:
    0       = North
    +pi/2   = East

Supported Dubins families:
    LSL
    RSR
    LSR
    RSL
    RLR
    LRL
"""

import math

import numpy as np


TWO_PI = 2.0 * math.pi


def mod2pi(angle):
    return angle % TWO_PI


def heading_between(first, second):
    """
    NED planar heading from first point to second point.
    """
    dn = second[0] - first[0]
    de = second[1] - first[1]

    return math.atan2(
        de,
        dn,
    )


def _candidate_paths(
    alpha,
    beta,
    distance,
):
    """
    Generate all mathematically valid Dubins candidates.

    Lengths are normalized by turning radius.
    """

    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)

    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)

    cos_alpha_beta = math.cos(
        alpha - beta
    )

    candidates = []

    # --------------------------------------------------------
    # LSL
    # --------------------------------------------------------

    p_squared = (
        2.0
        + distance**2
        - 2.0 * cos_alpha_beta
        + 2.0
        * distance
        * (
            sin_alpha
            - sin_beta
        )
    )

    if p_squared >= -1e-12:

        p = math.sqrt(
            max(
                0.0,
                p_squared,
            )
        )

        angle = math.atan2(
            cos_beta - cos_alpha,
            distance
            + sin_alpha
            - sin_beta,
        )

        candidates.append(
            (
                "LSL",
                mod2pi(
                    -alpha
                    + angle
                ),
                p,
                mod2pi(
                    beta
                    - angle
                ),
            )
        )

    # --------------------------------------------------------
    # RSR
    # --------------------------------------------------------

    p_squared = (
        2.0
        + distance**2
        - 2.0 * cos_alpha_beta
        + 2.0
        * distance
        * (
            -sin_alpha
            + sin_beta
        )
    )

    if p_squared >= -1e-12:

        p = math.sqrt(
            max(
                0.0,
                p_squared,
            )
        )

        angle = math.atan2(
            cos_alpha - cos_beta,
            distance
            - sin_alpha
            + sin_beta,
        )

        candidates.append(
            (
                "RSR",
                mod2pi(
                    alpha
                    - angle
                ),
                p,
                mod2pi(
                    -beta
                    + angle
                ),
            )
        )

    # --------------------------------------------------------
    # LSR
    # --------------------------------------------------------

    p_squared = (
        -2.0
        + distance**2
        + 2.0 * cos_alpha_beta
        + 2.0
        * distance
        * (
            sin_alpha
            + sin_beta
        )
    )

    if p_squared >= -1e-12:

        p = math.sqrt(
            max(
                0.0,
                p_squared,
            )
        )

        angle = (
            math.atan2(
                -cos_alpha
                - cos_beta,
                distance
                + sin_alpha
                + sin_beta,
            )
            - math.atan2(
                -2.0,
                math.sqrt(
                    max(
                        0.0,
                        p_squared,
                    )
                ),
            )
        )

        candidates.append(
            (
                "LSR",
                mod2pi(
                    -alpha
                    + angle
                ),
                p,
                mod2pi(
                    -beta
                    + angle
                ),
            )
        )

    # --------------------------------------------------------
    # RSL
    # --------------------------------------------------------

    p_squared = (
        -2.0
        + distance**2
        + 2.0 * cos_alpha_beta
        - 2.0
        * distance
        * (
            sin_alpha
            + sin_beta
        )
    )

    if p_squared >= -1e-12:

        p = math.sqrt(
            max(
                0.0,
                p_squared,
            )
        )

        angle = (
            math.atan2(
                cos_alpha
                + cos_beta,
                distance
                - sin_alpha
                - sin_beta,
            )
            - math.atan2(
                2.0,
                math.sqrt(
                    max(
                        0.0,
                        p_squared,
                    )
                ),
            )
        )

        candidates.append(
            (
                "RSL",
                mod2pi(
                    alpha
                    - angle
                ),
                p,
                mod2pi(
                    beta
                    - angle
                ),
            )
        )

    # --------------------------------------------------------
    # RLR
    # --------------------------------------------------------

    value = (
        6.0
        - distance**2
        + 2.0 * cos_alpha_beta
        + 2.0
        * distance
        * (
            sin_alpha
            - sin_beta
        )
    ) / 8.0

    if abs(value) <= 1.0 + 1e-12:

        value = min(
            1.0,
            max(
                -1.0,
                value,
            ),
        )

        p = mod2pi(
            TWO_PI
            - math.acos(
                value
            )
        )

        angle = math.atan2(
            cos_alpha - cos_beta,
            distance
            - sin_alpha
            + sin_beta,
        )

        t = mod2pi(
            alpha
            - angle
            + p / 2.0
        )

        q = mod2pi(
            alpha
            - beta
            - t
            + p
        )

        candidates.append(
            (
                "RLR",
                t,
                p,
                q,
            )
        )

    # --------------------------------------------------------
    # LRL
    # --------------------------------------------------------

    value = (
        6.0
        - distance**2
        + 2.0 * cos_alpha_beta
        + 2.0
        * distance
        * (
            -sin_alpha
            + sin_beta
        )
    ) / 8.0

    if abs(value) <= 1.0 + 1e-12:

        value = min(
            1.0,
            max(
                -1.0,
                value,
            ),
        )

        p = mod2pi(
            TWO_PI
            - math.acos(
                value
            )
        )

        angle = math.atan2(
            cos_alpha - cos_beta,
            distance
            + sin_alpha
            - sin_beta,
        )

        t = mod2pi(
            -alpha
            - angle
            + p / 2.0
        )

        q = mod2pi(
            beta
            - alpha
            - t
            + p
        )

        candidates.append(
            (
                "LRL",
                t,
                p,
                q,
            )
        )

    return candidates


def _advance(
    state,
    mode,
    distance,
    turning_radius,
):
    """
    Advance along one Dubins primitive.
    """

    north, east, heading = state

    if mode == "S":

        return np.array(
            [
                north
                + distance
                * math.cos(
                    heading
                ),

                east
                + distance
                * math.sin(
                    heading
                ),

                heading,
            ],
            dtype=float,
        )

    angle = (
        distance
        / turning_radius
    )

    if mode == "L":

        next_heading = (
            heading
            + angle
        )

        return np.array(
            [
                north
                + turning_radius
                * (
                    math.sin(
                        next_heading
                    )
                    - math.sin(
                        heading
                    )
                ),

                east
                + turning_radius
                * (
                    math.cos(
                        heading
                    )
                    - math.cos(
                        next_heading
                    )
                ),

                next_heading,
            ],
            dtype=float,
        )

    if mode == "R":

        next_heading = (
            heading
            - angle
        )

        return np.array(
            [
                north
                + turning_radius
                * (
                    math.sin(
                        heading
                    )
                    - math.sin(
                        next_heading
                    )
                ),

                east
                + turning_radius
                * (
                    math.cos(
                        next_heading
                    )
                    - math.cos(
                        heading
                    )
                ),

                next_heading,
            ],
            dtype=float,
        )

    raise ValueError(
        f"Unknown Dubins mode: {mode}"
    )


def generate_dubins_path(
    start,
    goal,
    turning_radius,
    step=0.5,
):
    """
    Generate the shortest forward-only Dubins path.

    start / goal:
        (north, east, heading)

    Returns:
        samples:
            numpy array [N, 3]

        information:
            dict containing selected family and path length
    """

    start = np.asarray(
        start,
        dtype=float,
    )

    goal = np.asarray(
        goal,
        dtype=float,
    )

    if (
        start.shape != (3,)
        or
        goal.shape != (3,)
    ):
        raise ValueError(
            "start and goal must be "
            "(north, east, heading)"
        )

    if turning_radius <= 0.0:
        raise ValueError(
            "turning_radius must be > 0"
        )

    if step <= 0.0:
        raise ValueError(
            "step must be > 0"
        )

    dn = (
        goal[0]
        - start[0]
    )

    de = (
        goal[1]
        - start[1]
    )

    direct_distance = math.hypot(
        dn,
        de,
    )

    theta = math.atan2(
        de,
        dn,
    )

    alpha = mod2pi(
        start[2]
        - theta
    )

    beta = mod2pi(
        goal[2]
        - theta
    )

    normalized_distance = (
        direct_distance
        / turning_radius
    )

    candidates = _candidate_paths(
        alpha,
        beta,
        normalized_distance,
    )

    if not candidates:
        raise RuntimeError(
            "No valid Dubins path found"
        )

    best = min(
        candidates,
        key=lambda candidate:
            sum(
                candidate[1:]
            ),
    )

    path_type = best[0]

    normalized_lengths = (
        best[1],
        best[2],
        best[3],
    )

    state = start.copy()

    samples = [
        state.copy()
    ]

    for (
        mode,
        normalized_length,
    ) in zip(
        path_type,
        normalized_lengths,
    ):

        segment_length = (
            normalized_length
            * turning_radius
        )

        if segment_length <= 1e-12:
            continue

        sample_count = max(
            1,
            int(
                math.ceil(
                    segment_length
                    / step
                )
            ),
        )

        increment = (
            segment_length
            / sample_count
        )

        for _ in range(
            sample_count
        ):

            state = _advance(
                state,
                mode,
                increment,
                turning_radius,
            )

            samples.append(
                state.copy()
            )

    samples = np.asarray(
        samples,
        dtype=float,
    )

    # Remove tiny numerical endpoint position error.
    samples[-1, 0] = goal[0]
    samples[-1, 1] = goal[1]

    information = {
        "type":
            path_type,

        "length":
            float(
                sum(
                    normalized_lengths
                )
                * turning_radius
            ),

        "turning_radius":
            float(
                turning_radius
            ),
    }

    return (
        samples,
        information,
    )


def waypoint_headings(
    points,
):
    """
    Assign tangent headings to a waypoint sequence.

    Endpoints follow their adjacent segment.

    Interior points use the chord from the previous waypoint to the
    following waypoint, producing a useful continuous tangent target
    for Dubins smoothing.
    """

    points = [
        (
            float(point[0]),
            float(point[1]),
        )
        for point in points
    ]

    if len(points) < 2:
        raise ValueError(
            "At least two waypoints are required"
        )

    headings = []

    for index in range(
        len(points)
    ):

        if index == 0:

            heading = heading_between(
                points[0],
                points[1],
            )

        elif index == (
            len(points) - 1
        ):

            heading = heading_between(
                points[-2],
                points[-1],
            )

        else:

            heading = heading_between(
                points[index - 1],
                points[index + 1],
            )

        headings.append(
            heading
        )

    return headings


def generate_dubins_through_waypoints(
    points,
    turning_radius=5.0,
    step=0.5,
):
    """
    Connect a waypoint sequence with tangent-continuous Dubins paths.

    Returns:
        full_path:
            numpy array [N, 3]

        segment_info:
            diagnostic information for each Dubins connection
    """

    if len(points) < 2:
        raise ValueError(
            "At least two waypoints are required"
        )

    headings = waypoint_headings(
        points
    )

    states = [
        (
            float(point[0]),
            float(point[1]),
            float(heading),
        )
        for point, heading in zip(
            points,
            headings,
        )
    ]

    combined = []
    segment_info = []

    for index in range(
        len(states) - 1
    ):

        samples, information = (
            generate_dubins_path(
                states[index],
                states[index + 1],
                turning_radius=turning_radius,
                step=step,
            )
        )

        information = dict(
            information
        )

        information[
            "segment"
        ] = index

        segment_info.append(
            information
        )

        if index == 0:

            combined.extend(
                samples
            )

        else:

            combined.extend(
                samples[1:]
            )

    return (
        np.asarray(
            combined,
            dtype=float,
        ),
        segment_info,
    )


def path_length(
    path,
):
    """
    Metric length of a sampled NED path.
    """

    if len(path) < 2:
        return 0.0

    differences = (
        path[1:, :2]
        - path[:-1, :2]
    )

    return float(
        np.sum(
            np.hypot(
                differences[:, 0],
                differences[:, 1],
            )
        )
    )


def validate_path_on_safe_grid(
    path,
    safe_occupancy,
    clearance,
    metadata,
):
    """
    Validate a sampled continuous path against the WAM-V-safe grid.

    Checks:
        - every sample is inside the map
        - every sample is in a safe cell
        - every grid cell crossed between consecutive samples is safe

    Returns a diagnostic dictionary.
    """

    from platoon_planner.sydney_environment import ned_to_grid
    from platoon_planner.astar_router import supercover_line_cells

    minimum_clearance = float("inf")
    checked_cells = set()

    previous_cell = None

    for sample_index, sample in enumerate(path):

        north = float(sample[0])
        east = float(sample[1])

        cell = ned_to_grid(
            north,
            east,
            metadata,
        )

        row, column = cell

        if not (
            0 <= row < safe_occupancy.shape[0]
            and
            0 <= column < safe_occupancy.shape[1]
        ):
            return {
                "safe": False,
                "reason": "outside_map",
                "sample_index": sample_index,
                "cell": cell,
                "minimum_clearance": None,
            }

        if previous_cell is None:
            segment_cells = [cell]
        else:
            segment_cells = supercover_line_cells(
                previous_cell,
                cell,
            )

        for checked_cell in segment_cells:

            if checked_cell in checked_cells:
                continue

            checked_cells.add(
                checked_cell
            )

            r, c = checked_cell

            if not (
                0 <= r < safe_occupancy.shape[0]
                and
                0 <= c < safe_occupancy.shape[1]
            ):
                return {
                    "safe": False,
                    "reason": "outside_map",
                    "sample_index": sample_index,
                    "cell": checked_cell,
                    "minimum_clearance": None,
                }

            if safe_occupancy[
                checked_cell
            ] != 0:

                return {
                    "safe": False,
                    "reason": "unsafe_cell",
                    "sample_index": sample_index,
                    "cell": checked_cell,
                    "minimum_clearance":
                        minimum_clearance
                        if minimum_clearance != float("inf")
                        else None,
                }

            minimum_clearance = min(
                minimum_clearance,
                float(
                    clearance[
                        checked_cell
                    ]
                ),
            )

        previous_cell = cell

    return {
        "safe": True,
        "reason": "ok",
        "sample_index": None,
        "cell": None,
        "minimum_clearance":
            minimum_clearance,
        "checked_cells":
            len(checked_cells),
    }


def generate_dubins_via_waypoints(
    points,
    start_heading,
    goal_heading,
    turning_radius,
    step=0.5,
):
    """
    Dubins-smooth a waypoint route while preserving the requested
    heading at the beginning and end.

    Interior waypoint headings follow the chord between their
    neighbouring waypoints.
    """

    if len(points) < 2:
        raise ValueError(
            "At least two waypoints are required"
        )

    points = [
        (
            float(point[0]),
            float(point[1]),
        )
        for point in points
    ]

    headings = []

    for index in range(len(points)):

        if index == 0:
            heading = float(start_heading)

        elif index == len(points) - 1:
            heading = float(goal_heading)

        else:
            heading = heading_between(
                points[index - 1],
                points[index + 1],
            )

        headings.append(heading)

    states = [
        (
            point[0],
            point[1],
            heading,
        )
        for point, heading
        in zip(points, headings)
    ]

    combined = []
    info = []

    for index in range(
        len(states) - 1
    ):

        samples, segment_info = (
            generate_dubins_path(
                states[index],
                states[index + 1],
                turning_radius=turning_radius,
                step=step,
            )
        )

        segment_info = dict(
            segment_info
        )

        segment_info["segment"] = index

        info.append(
            segment_info
        )

        if index == 0:
            combined.extend(samples)
        else:
            combined.extend(samples[1:])

    return (
        np.asarray(
            combined,
            dtype=float,
        ),
        info,
    )
