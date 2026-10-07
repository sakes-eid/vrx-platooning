#!/usr/bin/env python3

"""
Interactive ROS coverage planner for R1.

Workflow:
    1. Wait for live /r1/vehicle_state.
    2. Build the 3D-aware Sydney safe-water map.
    3. Let the user select two opposite coverage corners.
    4. Ask for line spacing.
    5. Build:
           live R1 pose
           -> A*
           -> Dubins approach
           -> coverage mission
    6. Feed that path into the existing R1 planner machinery.

The existing controller-facing planner topics are preserved.
"""

import rclpy

from std_msgs.msg import Int32

from platoon_interfaces.msg import VehicleState

from platoon_planner.stress_course_planner import (
    StressCoursePlanner,
)

from platoon_planner.sydney_environment import (
    find_sydney_mesh,
    load_shore_vertices,
    load_shore_triangles,
    build_safe_occupancy,
    wamv_navigation_vertical_band,
    build_vertical_slab_occupancy_grid,
)

from platoon_planner.coverage_router import (
    select_coverage_rectangle,
    build_coverage_mission_plan,
)

from platoon_planner.mission_preview import (
    preview_and_approve,
)


class CoverageMissionPlanner(
    StressCoursePlanner
):

    def __init__(self):

        self.building_mission = False

        super().__init__()

        self.get_logger().info(
            "R1 coverage planner waiting for "
            "live /r1/vehicle_state."
        )

    def ask_line_spacing(self):

        while True:

            try:

                spacing = float(
                    input(
                        "Enter coverage line spacing [m]: "
                    )
                )

                if spacing <= 0.0:
                    raise ValueError

                return spacing

            except ValueError:

                print(
                    "Line spacing must be "
                    "a positive number."
                )

    def state_callback(
        self,
        msg: VehicleState,
    ):

        self.r1 = msg

        if (
            self.initialized
            or self.building_mission
        ):
            return

        self.building_mission = True

        try:

            start_north = float(
                msg.x
            )

            start_east = float(
                msg.y
            )

            start_heading = float(
                msg.body_yaw
            )

            print()
            print("=" * 60)
            print("R1 COVERAGE MISSION SETUP")
            print("=" * 60)

            print(
                "Live R1 start:"
            )

            print(
                "  North   :",
                round(
                    start_north,
                    3,
                ),
                "m",
            )

            print(
                "  East    :",
                round(
                    start_east,
                    3,
                ),
                "m",
            )

            print(
                "  Heading :",
                round(
                    start_heading,
                    4,
                ),
                "rad",
            )

            print()
            print(
                "Building 3D-aware "
                "Sydney navigation map..."
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

            (
                safe_grid,
                clearance,
                required,
            ) = build_safe_occupancy(
                raw_grid,
                metadata,
                safety_margin=1.0,
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

            # =================================================
            # Select + build + preview + approve Coverage
            # =================================================

            while True:

                first, second = (
                    select_coverage_rectangle(
                        raw_grid,
                        safe_grid,
                        metadata,
                    )
                )

                print()
                print(
                    "Coverage area selected."
                )

                line_spacing = (
                    self.ask_line_spacing()
                )

                try:

                    mission = (
                        build_coverage_mission_plan(
                            first=first,
                            second=second,
                            line_spacing=line_spacing,
                            r1_start_ned=(
                                start_north,
                                start_east,
                            ),
                            r1_start_heading=(
                                start_heading
                            ),
                            raw_grid=raw_grid,
                            safe_grid=safe_grid,
                            clearance=clearance,
                            metadata=metadata,
                        )
                    )

                except (
                    RuntimeError,
                    ValueError,
                ) as exc:

                    print()
                    print("=" * 60)
                    print("COVERAGE MISSION REJECTED")
                    print("=" * 60)

                    print(
                        str(
                            exc
                        )
                    )

                    print()
                    print(
                        "Select another coverage area."
                    )

                    continue

                decision = (
                    preview_and_approve(
                        mission=mission,
                        raw_grid=raw_grid,
                        safe_grid=safe_grid,
                        metadata=metadata,
                        r1_start_ned=(
                            start_north,
                            start_east,
                        ),
                        important_points=[
                            {
                                "label":
                                    "Preferred Start Corner",

                                "point":
                                    first,

                                "marker":
                                    "x",
                            },
                            {
                                "label":
                                    "Opposite Corner",

                                "point":
                                    second,

                                "marker":
                                    "x",
                            },
                        ],
                    )
                )

                if decision == "modify":

                    print()
                    print(
                        "Reopening Coverage "
                        "mission selector..."
                    )

                    continue

                if decision == "cancel":

                    print()
                    print("=" * 60)
                    print("MISSION CANCELLED")
                    print("=" * 60)

                    print(
                        "No reference path "
                        "has been published."
                    )

                    self.get_logger().info(
                        "Coverage mission cancelled "
                        "before publication."
                    )

                    return

                # ACCEPT
                break

            self.mission_plan = mission

            self.path_points = list(
                mission.path_points
            )

            if len(
                self.path_points
            ) < 2:

                raise RuntimeError(
                    "Coverage mission contains "
                    "fewer than two points."
                )

            # Reuse the proven R1 planner machinery.
            self.path_s = (
                self.cumulative_lengths(
                    self.path_points
                )
            )

            self.curvature = (
                self.estimate_curvature(
                    self.path_points
                )
            )

            self.speed_profile = (
                self.build_speed_profile(
                    self.path_points,
                    self.path_s,
                    self.curvature,
                )
            )

            self.controller_active_index = 1

            self.initialized = True

            # Publish the same interface the R1
            # controller already expects.
            self.publish_full_path()

            count = Int32()
            count.data = len(
                self.path_points
            )

            self.waypoint_count_pub.publish(
                count
            )

            print()
            print("=" * 60)
            print("COVERAGE MISSION READY")
            print("=" * 60)

            print(
                "A* waypoints       :",
                mission.diagnostics[
                    "approach_astar_waypoints"
                ],
            )

            print(
                "Coverage segments  :",
                mission.diagnostics[
                    "coverage_segments"
                ],
            )

            print(
                "Mission samples    :",
                mission.waypoint_count,
            )

            print(
                "Mission length     :",
                f"{mission.path_length:.2f} m",
            )

            print(
                "Minimum clearance  :",
                f"{mission.minimum_clearance:.2f} m",
            )

            print(
                "Collision check    : PASS"
            )

            print(
                "Publishing         : "
                "/planner/reference_path"
            )

            print(
                "R1 controller may now "
                "begin the mission."
            )

            print("=" * 60)
            print()

            self.get_logger().info(
                "Coverage mission published: "
                f"{len(self.path_points)} points, "
                f"{self.path_s[-1]:.1f} m."
            )

        except Exception as exc:

            self.get_logger().error(
                "Coverage mission setup failed: "
                f"{exc}"
            )

            raise

        finally:

            self.building_mission = False


def main(args=None):

    rclpy.init(
        args=args
    )

    node = CoverageMissionPlanner()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
