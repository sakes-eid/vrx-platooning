#!/usr/bin/env python3

"""
Interactive ROS straight-mission planner for R1.

Workflow:
    1. Wait for live /r1/vehicle_state.
    2. Build the 3D-aware Sydney navigation map.
    3. Let the user select Straight START and END.
    4. Reject unsafe straight sections.
    5. Build:
           live R1
           -> A*
           -> Dubins
           -> Straight START
           -> exact straight
           -> Straight END
    6. Feed the completed mission into the existing R1
       speed / curvature / lookahead planner machinery.

The controller-facing ROS interface is unchanged.
"""

import rclpy

from std_msgs.msg import Int32

from platoon_interfaces.msg import (
    VehicleState,
)

from platoon_planner.stress_course_planner import (
    StressCoursePlanner,
)

from platoon_planner.sydney_environment import (
    find_sydney_mesh,
    load_shore_vertices,
    load_shore_triangles,
    wamv_navigation_vertical_band,
    build_vertical_slab_occupancy_grid,
    build_safe_occupancy,
)

from platoon_planner.mission_point_selector import (
    select_mission_points,
)

from platoon_planner.straight_mission import (
    build_straight_mission_plan,
)

from platoon_planner.mission_preview import (
    preview_and_approve,
)


class StraightMissionPlanner(
    StressCoursePlanner
):

    def __init__(
        self,
    ):

        self.building_mission = False

        self.mission_plan = None

        super().__init__()

        self.get_logger().info(
            "R1 straight planner waiting for "
            "live /r1/vehicle_state."
        )

    def state_callback(
        self,
        msg: VehicleState,
    ):

        self.r1 = msg

        if (
            self.initialized
            or
            self.building_mission
        ):
            return

        self.building_mission = True

        try:

            # =================================================
            # Live R1 state
            # =================================================

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
            print("R1 STRAIGHT MISSION SETUP")
            print("=" * 60)

            print(
                f"Live R1 North   : "
                f"{start_north:.3f} m"
            )

            print(
                f"Live R1 East    : "
                f"{start_east:.3f} m"
            )

            print(
                f"Live R1 Heading : "
                f"{start_heading:.4f} rad"
            )

            # =================================================
            # Sydney navigation map
            # =================================================

            print()
            print(
                "Building 3D-aware Sydney "
                "navigation map..."
            )

            mesh = find_sydney_mesh()

            vertices = (
                load_shore_vertices(
                    mesh
                )
            )

            triangles = (
                load_shore_triangles(
                    mesh
                )
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
                required_clearance,
            ) = build_safe_occupancy(
                raw_grid,
                metadata,
                safety_margin=1.0,
            )

            print(
                f"Map ready. Required clearance: "
                f"{required_clearance:.3f} m"
            )

            print()
            print(
                "White  = safe water"
            )

            print(
                "Orange = WAM-V safety buffer"
            )

            print(
                "Black  = collision terrain"
            )

            # =================================================
            # Mission selection + preview + approval
            # =================================================

            while True:

                selected = (
                    select_mission_points(
                        raw_grid=raw_grid,
                        safe_grid=safe_grid,
                        metadata=metadata,
                        labels=(
                            "Straight Start",
                            "Straight End",
                        ),
                        title=(
                            "Select Straight Mission"
                        ),
                    )
                )

                selected_start = (
                    selected[0]
                )

                selected_end = (
                    selected[1]
                )

                try:

                    mission = (
                        build_straight_mission_plan(
                            selected_start=(
                                selected_start
                            ),
                            selected_end=(
                                selected_end
                            ),
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

                except RuntimeError as exc:

                    print()
                    print("=" * 60)
                    print("STRAIGHT MISSION REJECTED")
                    print("=" * 60)

                    print(
                        str(
                            exc
                        )
                    )

                    print()
                    print(
                        "Select another "
                        "START and END."
                    )

                    continue

                # =============================================
                # Common mission preview
                # =============================================

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
                                    "Straight Start",

                                "point":
                                    selected_start,

                                "marker":
                                    "x",
                            },
                            {
                                "label":
                                    "Straight End",

                                "point":
                                    selected_end,

                                "marker":
                                    "x",
                            },
                        ],
                    )
                )

                if decision == "modify":

                    print()
                    print(
                        "Reopening Straight "
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
                        "Straight mission cancelled "
                        "before publication."
                    )

                    return

                # ACCEPT
                break

            self.mission_plan = mission

            # =================================================
            # Feed common MissionPlan into the proven
            # R1 planner machinery.
            # =================================================

            self.path_points = list(
                mission.path_points
            )

            if len(
                self.path_points
            ) < 2:

                raise RuntimeError(
                    "Straight mission contains "
                    "fewer than two path points."
                )

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

            # =================================================
            # Publish existing controller interface
            # =================================================

            self.publish_full_path()

            count = Int32()

            count.data = len(
                self.path_points
            )

            self.waypoint_count_pub.publish(
                count
            )

            # =================================================
            # Report
            # =================================================

            print()
            print("=" * 60)
            print("STRAIGHT MISSION READY")
            print("=" * 60)

            print(
                "Mission samples    :",
                mission.waypoint_count,
            )

            print(
                "Complete length    :",
                f"{mission.path_length:.2f} m",
            )

            print(
                "Straight length    :",
                (
                    f"{mission.diagnostics['straight_length']:.2f} m"
                ),
            )

            print(
                "A* waypoints       :",
                mission.diagnostics[
                    "astar_waypoints"
                ],
            )

            print(
                "Minimum clearance  :",
                f"{mission.minimum_clearance:.2f} m",
            )

            print(
                "Spawn escape       :",
                mission.diagnostics[
                    "spawn_escape_used"
                ],
            )

            print(
                "Maximum speed      :",
                f"{self.max_speed:.2f} m/s",
            )

            print(
                "Collision checked  :",
                mission.collision_checked,
            )

            print()
            print(
                "Reference path published."
            )

            print(
                "R1 controller may now "
                "execute the mission."
            )

            print("=" * 60)

            self.get_logger().info(
                "Straight mission published: "
                f"{len(self.path_points)} points, "
                f"{self.path_s[-1]:.1f} m, "
                f"R1 ceiling={self.max_speed:.2f} m/s."
            )

        except Exception as exc:

            self.get_logger().error(
                f"Straight mission setup failed: "
                f"{exc}"
            )

            self.building_mission = False

            raise


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = StraightMissionPlanner()

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
