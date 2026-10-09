#!/usr/bin/env python3

"""
Pure standardized Stress mission geometry.

No ROS node is required.

This is the authoritative generator used by:
    - StressCoursePlanner
    - saved mission regeneration
"""

import math

from platoon_planner.mission_definition import (
    STANDARD_STRESS_COURSE,
)

from platoon_planner.mission_plan import (
    MissionPlan,
)


def _distance(
    first,
    second,
):
    return math.hypot(
        second[0] - first[0],
        second[1] - first[1],
    )


def _wrap_pi(
    value,
):
    return math.atan2(
        math.sin(value),
        math.cos(value),
    )


def _append_straight(
    points,
    pose,
    length,
    point_spacing,
):
    north, east, heading = pose

    count = max(
        1,
        int(
            math.ceil(
                length / point_spacing
            )
        ),
    )

    start_north = north
    start_east = east

    for index in range(
        1,
        count + 1,
    ):

        distance = (
            length
            * index
            / count
        )

        point = (
            start_north
            + distance
            * math.cos(
                heading
            ),

            start_east
            + distance
            * math.sin(
                heading
            ),
        )

        if (
            _distance(
                points[-1],
                point,
            )
            > 1e-6
        ):

            points.append(
                point
            )

    return (
        start_north
        + length
        * math.cos(
            heading
        ),

        start_east
        + length
        * math.sin(
            heading
        ),

        heading,
    )


def _append_arc(
    points,
    pose,
    radius,
    turn_angle,
    point_spacing,
):
    north, east, heading = pose

    sign = (
        1.0
        if turn_angle >= 0.0
        else -1.0
    )

    normal_north = (
        -math.sin(
            heading
        )
    )

    normal_east = (
        math.cos(
            heading
        )
    )

    center_north = (
        north
        + sign
        * radius
        * normal_north
    )

    center_east = (
        east
        + sign
        * radius
        * normal_east
    )

    radial_north = (
        north
        - center_north
    )

    radial_east = (
        east
        - center_east
    )

    arc_length = abs(
        radius
        * turn_angle
    )

    count = max(
        2,
        int(
            math.ceil(
                arc_length
                / point_spacing
            )
        ),
    )

    for index in range(
        1,
        count + 1,
    ):

        angle = (
            turn_angle
            * index
            / count
        )

        cosine = math.cos(
            angle
        )

        sine = math.sin(
            angle
        )

        rotated_north = (
            cosine
            * radial_north
            - sine
            * radial_east
        )

        rotated_east = (
            sine
            * radial_north
            + cosine
            * radial_east
        )

        point = (
            center_north
            + rotated_north,

            center_east
            + rotated_east,
        )

        if (
            _distance(
                points[-1],
                point,
            )
            > 1e-6
        ):

            points.append(
                point
            )

    end_heading = _wrap_pi(
        heading
        + turn_angle
    )

    return (
        points[-1][0],
        points[-1][1],
        end_heading,
    )


def build_standard_stress_mission_plan(
    start_ned,
    point_spacing=0.50,
    straight_length=80.0,
    turn_90_radius=5.0,
    semicircle_small_radius=6.0,
    semicircle_large_radius=10.0,
    coverage_lane_length=14.0,
    coverage_tight_radius=5.0,
    coverage_wide_radius=9.0,
):
    """
    Build the fixed standard Stress course.

    Initial heading is due North, matching the existing
    StressCoursePlanner behaviour.
    """

    start_north = float(
        start_ned[0]
    )

    start_east = float(
        start_ned[1]
    )

    points = [
        (
            start_north,
            start_east,
        )
    ]

    pose = (
        start_north,
        start_east,
        0.0,
    )

    pose = _append_straight(
        points,
        pose,
        straight_length,
        point_spacing,
    )

    pose = _append_arc(
        points,
        pose,
        turn_90_radius,
        math.radians(
            90.0
        ),
        point_spacing,
    )

    pose = _append_arc(
        points,
        pose,
        semicircle_small_radius,
        math.radians(
            180.0
        ),
        point_spacing,
    )

    pose = _append_arc(
        points,
        pose,
        semicircle_large_radius,
        math.radians(
            180.0
        ),
        point_spacing,
    )

    pose = _append_straight(
        points,
        pose,
        8.0,
        point_spacing,
    )

    pose = _append_straight(
        points,
        pose,
        coverage_lane_length,
        point_spacing,
    )

    pose = _append_arc(
        points,
        pose,
        coverage_tight_radius,
        math.radians(
            180.0
        ),
        point_spacing,
    )

    pose = _append_straight(
        points,
        pose,
        coverage_lane_length,
        point_spacing,
    )

    pose = _append_arc(
        points,
        pose,
        coverage_wide_radius,
        math.radians(
            -180.0
        ),
        point_spacing,
    )

    _append_straight(
        points,
        pose,
        coverage_lane_length,
        point_spacing,
    )

    mission = MissionPlan(
        mission_type="stress",

        path_points=points,

        parameters={
            "standard_course":
                STANDARD_STRESS_COURSE,

            "standardized":
                True,

            "start_ned":
                (
                    start_north,
                    start_east,
                ),

            "point_spacing":
                float(
                    point_spacing
                ),

            "straight_length":
                float(
                    straight_length
                ),

            "turn_90_radius":
                float(
                    turn_90_radius
                ),

            "semicircle_small_radius":
                float(
                    semicircle_small_radius
                ),

            "semicircle_large_radius":
                float(
                    semicircle_large_radius
                ),

            "coverage_lane_length":
                float(
                    coverage_lane_length
                ),

            "coverage_tight_radius":
                float(
                    coverage_tight_radius
                ),

            "coverage_wide_radius":
                float(
                    coverage_wide_radius
                ),
        },

        minimum_clearance=None,

        collision_checked=False,

        diagnostics={
            "geometry_editable":
                False,

            "standard_course":
                True,
        },
    )

    mission.validate()

    return mission
