#!/usr/bin/env python3

import rclpy

from std_msgs.msg import Int32
from platoon_interfaces.msg import VehicleState

from platoon_planner.stress_course_planner import StressCoursePlanner

from platoon_planner.sydney_environment import (
    find_sydney_mesh,
    load_shore_vertices,
    load_shore_triangles,
    wamv_navigation_vertical_band,
    build_vertical_slab_occupancy_grid,
    build_safe_occupancy,
)

from platoon_planner.mission_storage import (
    load_mission_definition,
)

from platoon_planner.mission_factory import (
    build_plan_from_definition,
)

from platoon_planner.mission_preview import (
    preview_and_approve,
)


class SavedMissionPlanner(StressCoursePlanner):

    def __init__(self):

        self.building_mission = False
        self.mission_plan = None

        super().__init__()

        self.declare_parameter(
            "mission_file",
            "",
        )

        self.mission_file = str(
            self.get_parameter(
                "mission_file"
            ).value
        ).strip()

        if not self.mission_file:
            raise RuntimeError(
                "mission_file parameter is required."
            )

        self.get_logger().info(
            "Saved mission planner waiting for "
            "live /r1/vehicle_state."
        )

    def state_callback(
        self,
        msg: VehicleState,
    ):

        self.r1 = msg

        if (
            self.initialized
            or self.building_mission
            or self.mission_cancelled
        ):
            return

        self.building_mission = True

        try:

            start_north = float(msg.x)
            start_east = float(msg.y)
            start_heading = float(
                msg.body_yaw
            )

            print()
            print("=" * 60)
            print("SAVED MISSION SETUP")
            print("=" * 60)
            print(
                "Mission file:",
                self.mission_file,
            )

            definition = (
                load_mission_definition(
                    self.mission_file
                )
            )

            # -------------------------------------------------
            # Sydney map
            # -------------------------------------------------

            print(
                "Building Sydney navigation map..."
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

            # -------------------------------------------------
            # Fresh executable mission from current R1
            # -------------------------------------------------

            mission = (
                build_plan_from_definition(
                    definition=definition,
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
                    important_points=None,
                    allow_modify=False,
                )
            )

            if decision == "cancel":

                self.mission_cancelled = True

                print()
                print(
                    "Saved mission cancelled."
                )

                return

            # -------------------------------------------------
            # Feed MissionPlan into normal R1 planner machinery
            # -------------------------------------------------

            self.mission_plan = mission

            self.path_points = list(
                mission.path_points
            )

            if len(self.path_points) < 2:
                raise RuntimeError(
                    "Saved mission contains fewer "
                    "than two path points."
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
            print("SAVED MISSION READY")
            print("=" * 60)
            print(
                "Name               :",
                definition.name,
            )
            print(
                "Type               :",
                definition.mission_type,
            )
            print(
                "Mission samples    :",
                mission.waypoint_count,
            )
            print(
                "Complete length    :",
                f"{mission.path_length:.2f} m",
            )
            print(
                "Maximum speed      :",
                f"{self.max_speed:.2f} m/s",
            )
            print(
                "Collision checked  :",
                mission.collision_checked,
            )
            print(
                "Reference path published."
            )
            print("=" * 60)

        except Exception as exc:

            self.get_logger().error(
                f"Saved mission setup failed: {exc}"
            )

            raise

        finally:

            self.building_mission = False


def main(args=None):

    rclpy.init(
        args=args
    )

    node = SavedMissionPlanner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
