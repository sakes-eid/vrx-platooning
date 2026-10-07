#!/usr/bin/env python3

"""
Common mission representation for every R1 mission type.

All mission generators must ultimately return MissionPlan so the
bringup, preview, logging, controller integration, and saved-mission
system do not need mission-specific interfaces.

Coordinate convention:
    world_ned
    x = North
    y = East
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import math


Point = Tuple[
    float,
    float,
]


@dataclass
class MissionPlan:

    mission_type: str

    # Final chronological world_ned path followed by R1.
    path_points: List[Point]

    # User-selected / mission-specific inputs.
    parameters: Dict[
        str,
        Any,
    ] = field(
        default_factory=dict
    )

    # Planner diagnostics.
    minimum_clearance: Optional[
        float
    ] = None

    collision_checked: bool = False

    diagnostics: Dict[
        str,
        Any,
    ] = field(
        default_factory=dict
    )

    def validate(self):

        if len(
            self.path_points
        ) < 2:

            raise ValueError(
                "Mission must contain at least "
                "two path points."
            )

        cleaned = []

        for point in self.path_points:

            if len(point) < 2:

                raise ValueError(
                    "Mission point must contain "
                    "North and East."
                )

            north = float(
                point[0]
            )

            east = float(
                point[1]
            )

            if not (
                math.isfinite(
                    north
                )
                and
                math.isfinite(
                    east
                )
            ):

                raise ValueError(
                    "Mission contains a "
                    "non-finite coordinate."
                )

            cleaned.append(
                (
                    north,
                    east,
                )
            )

        self.path_points = (
            cleaned
        )

    @property
    def waypoint_count(self):

        return len(
            self.path_points
        )

    @property
    def path_length(self):

        total = 0.0

        for index in range(
            1,
            len(
                self.path_points
            ),
        ):

            first = (
                self.path_points[
                    index - 1
                ]
            )

            second = (
                self.path_points[
                    index
                ]
            )

            total += math.hypot(
                second[0]
                - first[0],

                second[1]
                - first[1],
            )

        return total

    @property
    def start(self):

        return (
            self.path_points[0]
        )

    @property
    def goal(self):

        return (
            self.path_points[-1]
        )

    def summary(self):

        return {
            "mission_type":
                self.mission_type,

            "waypoint_count":
                self.waypoint_count,

            "path_length":
                self.path_length,

            "start":
                self.start,

            "goal":
                self.goal,

            "minimum_clearance":
                self.minimum_clearance,

            "collision_checked":
                self.collision_checked,
        }
