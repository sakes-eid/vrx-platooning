#!/usr/bin/env python3

"""
Persistent mission definition.

MissionDefinition stores the user-defined mission geometry only.

It deliberately does NOT store:
    - current R1 position
    - A* approach
    - Dubins approach
    - robot count
    - test/tuning mode
    - controller settings
    - run name
    - headless / RViz settings

Those belong to the current run.

A loaded mission is regenerated from the current live R1 state.
"""

from dataclasses import (
    dataclass,
    field,
)

from typing import (
    Any,
    Dict,
)


MISSION_SCHEMA_VERSION = 1

VALID_MISSION_TYPES = (
    "straight",
    "curve",
    "coverage",
    "stress",
)

COORDINATE_FRAME = "world_ned"

STANDARD_STRESS_COURSE = (
    "standard_stress_v1"
)


@dataclass
class MissionDefinition:

    name: str

    mission_type: str

    geometry: Dict[
        str,
        Any,
    ] = field(
        default_factory=dict
    )

    schema_version: int = (
        MISSION_SCHEMA_VERSION
    )

    coordinate_frame: str = (
        COORDINATE_FRAME
    )

    def validate(
        self,
    ):

        self.name = (
            self.name.strip()
        )

        self.mission_type = (
            self.mission_type
            .strip()
            .lower()
        )

        if not self.name:

            raise ValueError(
                "Mission name must not be empty."
            )

        if (
            self.schema_version
            != MISSION_SCHEMA_VERSION
        ):

            raise ValueError(
                "Unsupported mission schema version: "
                f"{self.schema_version}"
            )

        if (
            self.coordinate_frame
            != COORDINATE_FRAME
        ):

            raise ValueError(
                "Saved missions must use "
                "world_ned coordinates."
            )

        if (
            self.mission_type
            not in VALID_MISSION_TYPES
        ):

            raise ValueError(
                f"Unknown mission type: "
                f"{self.mission_type}"
            )

        if not isinstance(
            self.geometry,
            dict,
        ):

            raise ValueError(
                "geometry must be a dictionary."
            )

        if (
            self.mission_type
            == "straight"
        ):

            self._validate_point(
                "start"
            )

            self._validate_point(
                "end"
            )

        elif (
            self.mission_type
            == "curve"
        ):

            self._validate_point(
                "start"
            )

            self._validate_point(
                "apogee"
            )

            self._validate_point(
                "end"
            )

        elif (
            self.mission_type
            == "coverage"
        ):

            self._validate_point(
                "first_corner"
            )

            self._validate_point(
                "opposite_corner"
            )

            spacing = self.geometry.get(
                "line_spacing"
            )

            try:

                spacing = float(
                    spacing
                )

            except (
                TypeError,
                ValueError,
            ):

                raise ValueError(
                    "Coverage line_spacing "
                    "must be numeric."
                )

            if spacing <= 0.0:

                raise ValueError(
                    "Coverage line_spacing "
                    "must be > 0."
                )

            self.geometry[
                "line_spacing"
            ] = spacing

        elif (
            self.mission_type
            == "stress"
        ):

            course = self.geometry.get(
                "standard_course"
            )

            if (
                course
                != STANDARD_STRESS_COURSE
            ):

                raise ValueError(
                    "Stress mission must use "
                    f"{STANDARD_STRESS_COURSE}."
                )

    def _validate_point(
        self,
        key,
    ):

        point = self.geometry.get(
            key
        )

        if not isinstance(
            point,
            (
                list,
                tuple,
            ),
        ):

            raise ValueError(
                f"{key} must contain "
                "[North, East]."
            )

        if len(
            point
        ) != 2:

            raise ValueError(
                f"{key} must contain exactly "
                "North and East."
            )

        try:

            north = float(
                point[0]
            )

            east = float(
                point[1]
            )

        except (
            TypeError,
            ValueError,
        ):

            raise ValueError(
                f"{key} coordinates "
                "must be numeric."
            )

        # Normalize to JSON-friendly lists.
        self.geometry[
            key
        ] = [
            north,
            east,
        ]

    def to_dict(
        self,
    ):

        self.validate()

        return {
            "schema_version":
                self.schema_version,

            "name":
                self.name,

            "mission_type":
                self.mission_type,

            "coordinate_frame":
                self.coordinate_frame,

            "geometry":
                self.geometry,
        }

    @classmethod
    def from_dict(
        cls,
        data,
    ):

        if not isinstance(
            data,
            dict,
        ):

            raise ValueError(
                "Mission file root "
                "must be a JSON object."
            )

        mission = cls(
            name=data.get(
                "name",
                "",
            ),

            mission_type=data.get(
                "mission_type",
                "",
            ),

            geometry=data.get(
                "geometry",
                {},
            ),

            schema_version=data.get(
                "schema_version",
                -1,
            ),

            coordinate_frame=data.get(
                "coordinate_frame",
                "",
            ),
        )

        mission.validate()

        return mission
