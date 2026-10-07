#!/usr/bin/env python3

"""
Compatibility adapter for the coverage planner.

The authoritative A* + Dubins approach implementation now lives in:

    mission_approach.py

This wrapper preserves the existing coverage planner API.
"""

from platoon_planner.mission_approach import (
    build_mission_approach,
)


def build_approach_path(
    start_ned,
    start_heading,
    first_segment,
    line_spacing,
    safe_grid,
    clearance,
    metadata,
    raw_grid=None,
):
    """
    Preserve the original coverage approach interface.
    """

    return build_mission_approach(
        start_ned=start_ned,
        start_heading=start_heading,

        target_ned=(
            first_segment[
                "start_ned"
            ]
        ),

        target_heading=(
            first_segment[
                "heading"
            ]
        ),

        turning_radius=(
            float(line_spacing)
            / 2.0
        ),

        safe_grid=safe_grid,
        clearance=clearance,
        metadata=metadata,
        raw_grid=raw_grid,
    )
