import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int32, String

from platoon_interfaces.msg import VehicleState

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
)


def distance(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1],
    )


class FollowerPlanner(Node):

    FOLLOW = 'FOLLOW'
    FORMATION_BRAKE = 'FORMATION_BRAKE'
    FORMATION_HOLD = 'FORMATION_HOLD'
    PARK_REVERSE = 'PARK_REVERSE'
    PARK_ALIGN = 'PARK_ALIGN'
    PARK = 'PARK'
    DWELL = 'DWELL'
    SUCCESS = 'SUCCESS'

    def __init__(self):
        super().__init__('follower_planner')

        # ------------------------------------------------------
        # Generic predecessor -> follower pair configuration
        # ------------------------------------------------------
        self.declare_parameter('follower_id', 'r2')
        self.declare_parameter('predecessor_id', 'r1')

        # Kept configurable because R1 uses /experiment/success,
        # while later followers use their predecessor's success topic.
        self.declare_parameter(
            'predecessor_success_topic',
            '/experiment/success',
        )

        self.declare_parameter(
            'reference_path_topic',
            '/planner/reference_path',
        )

        # Terminal handoff source:
        #   reference_path      -> R2 <- R1 legacy/current behaviour
        #   predecessor_mission -> later followers such as R3 <- R2
        #
        # Later followers must not receive the global reference path.
        self.declare_parameter(
            'terminal_handoff_mode',
            'reference_path',
        )

        self.declare_parameter(
            'predecessor_mission_state_topic',
            '',
        )

        # Terminal behaviour after formation braking:
        #   parking -> preserve the existing two-robot experiment
        #   hold    -> remain stopped in formation for platooning
        self.declare_parameter(
            'terminal_behavior',
            'parking',
        )

        # Backward-compatible default for the current R2 test.
        # Later R3 launch will give each follower its own mission topic.
        self.declare_parameter(
            'mission_state_topic',
            '/platoon/mission_state',
        )

        self.declare_parameter(
            'mission_success_topic',
            '/platoon/success',
        )

        # Release policy:
        #   local_trail
        #       Original R2 behaviour. Wait until this predecessor
        #       has produced enough real breadcrumb history.
        #
        #   predecessor_release_bootstrap
        #       Later platoon members start when their predecessor
        #       is released. Initial breadcrumbs are constructed only
        #       from the current predecessor/follower pair geometry.
        #       No global/reference path information is used.
        self.declare_parameter(
            'release_mode',
            'local_trail',
        )

        self.declare_parameter(
            'predecessor_release_topic',
            '',
        )

        self.follower_id = str(
            self.get_parameter('follower_id').value
        )
        self.predecessor_id = str(
            self.get_parameter('predecessor_id').value
        )

        self.predecessor_success_topic = str(
            self.get_parameter(
                'predecessor_success_topic'
            ).value
        )

        self.reference_path_topic = str(
            self.get_parameter(
                'reference_path_topic'
            ).value
        )

        self.terminal_handoff_mode = str(
            self.get_parameter(
                'terminal_handoff_mode'
            ).value
        )

        configured_predecessor_mission_topic = str(
            self.get_parameter(
                'predecessor_mission_state_topic'
            ).value
        )

        self.predecessor_mission_state_topic = (
            configured_predecessor_mission_topic
            if configured_predecessor_mission_topic
            else f'/{self.predecessor_id}/mission_state'
        )

        if self.terminal_handoff_mode not in (
            'reference_path',
            'predecessor_mission',
        ):
            raise ValueError(
                'terminal_handoff_mode must be '
                "'reference_path' or 'predecessor_mission'"
            )

        self.terminal_behavior = str(
            self.get_parameter(
                'terminal_behavior'
            ).value
        )

        if self.terminal_behavior not in (
            'parking',
            'hold',
        ):
            raise ValueError(
                'terminal_behavior must be '
                "'parking' or 'hold'"
            )

        self.mission_state_topic = str(
            self.get_parameter(
                'mission_state_topic'
            ).value
        )

        self.mission_success_topic = str(
            self.get_parameter(
                'mission_success_topic'
            ).value
        )

        self.release_mode = str(
            self.get_parameter(
                'release_mode'
            ).value
        )

        if self.release_mode not in (
            'local_trail',
            'predecessor_release_bootstrap',
        ):
            raise ValueError(
                'release_mode must be '
                "'local_trail' or "
                "'predecessor_release_bootstrap'"
            )

        configured_release_topic = str(
            self.get_parameter(
                'predecessor_release_topic'
            ).value
        )

        self.predecessor_release_topic = (
            configured_release_topic
            if configured_release_topic
            else f'/planner/{self.predecessor_id}/released'
        )

        self.predecessor_state_topic = (
            f'/{self.predecessor_id}/vehicle_state'
        )
        self.follower_state_topic = (
            f'/{self.follower_id}/vehicle_state'
        )

        self.breadcrumb_topic = (
            f'/planner/{self.follower_id}/breadcrumb_path'
        )
        self.parking_topic = (
            f'/planner/{self.follower_id}/parking_path'
        )

        self.target_capture_topic = (
            f'/{self.predecessor_id}/target_capture'
        )
        self.reference_distance_topic = (
            f'/{self.follower_id}/following/reference_distance'
        )
        self.terminal_settled_topic = (
            f'/{self.follower_id}/terminal_settled'
        )
        self.follower_success_topic = (
            f'/{self.follower_id}/success'
        )
        self.clearance_topic = (
            f'/{self.follower_id}/safety/hitbox_clearance'
        )
        self.aligned_topic = (
            f'/{self.follower_id}/parking/aligned'
        )

        params = {
            # Hull-to-hull target is 5 m. For straight aligned
            # boats this corresponds to ~10.371 m reference lag.
            'formation_distance': 5.0,

            # Desired longitudinal formation is measured ALONG
            # the unsmoothed breadcrumb chain.
            #
            # 2.822 + 5.000 + 2.549 = 10.371 m reference lag.
            'predecessor_rear_extent': 2.822,
            'follower_front_extent': 2.549,
            'breadcrumb_lag_distance': 10.371,

            'breadcrumb_spacing': 0.20,
            'filter_alpha': 0.35,
            'jump_limit': 2.0,
            'minimum_extra_trail': 1.0,

            # FOLLOW path is a smooth local curve built from the
            # historical breadcrumb trail. The follower never aims
            # directly at an individual breadcrumb.
            'curve_samples_per_segment': 3.0,
            'curve_tangent_scale': 0.50,
            'curve_backtrack_points': 4.0,
            'passed_point_margin': 0.05,

            # Breadcrumb hierarchy:
            # acquire the correct trail segment ONCE while the
            # initial trail is still short, then progress only
            # chronologically through subsequent segments.
            'progress_capture_radius': 0.35,
            'progress_corridor_m': 3.5,
            'max_progress_advances_per_cycle': 8.0,

            'parking_separation': 18.0,
            'parking_escape_distance': 10.0,
            'parking_spacing': 0.5,

            # R1 target-capture handoff. This matches the leader's
            # final waypoint tolerance (1 m) and uses a very short
            # persistence check before latching the event.
            'target_capture_tolerance': 1.0,
            'target_capture_hold': 0.20,

            # Parking reverse is now conditional. If simultaneous
            # formation braking leaves >= 4.5 m hull clearance,
            # reverse is skipped entirely.
            'parking_reverse_clearance': 4.5,
            'parking_reverse_hold': 0.30,

            'dwell_time': 5.0,

            # V2.1 diagnostic only.
            'historical_preview_decel': 0.18,
            'historical_preview_max_speed': 1.5,
        }

        for name, default in params.items():
            self.declare_parameter(name, default)

        for name in params:
            setattr(
                self,
                name,
                float(self.get_parameter(name).value),
            )

        self.predecessor = None
        self.follower = None

        self.filtered_predecessor = None
        self.breadcrumbs = deque(maxlen=8000)
        self.breadcrumb_speeds = deque(maxlen=8000)

        # Monotonic progress through the leader's historical trail.
        # Once R2 passes a breadcrumb, this index never moves back.
        self.follower_progress_index = 0
        self.progress_acquired = False

        self.mode = self.FOLLOW
        self.previous_mode = None
        self.trail_ready = False

        self.predecessor_released = False
        self.release_bootstrap_complete = False

        self.leader_done = False
        self.follower_done = False

        self.hitbox_clearance = float('inf')
        self.parking_aligned = False

        # R1 final-target capture is intentionally separate from
        # R1 SUCCESS. Capture starts simultaneous formation braking;
        # SUCCESS only permits the later parking manoeuvre.
        self.final_target = None
        self.target_capture_candidate_start = None
        self.target_capture_latched = False
        self.follower_terminal_settled = False

        self.reverse_ready_start = None
        self.dwell_start = None

        self.parking_path = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # V2.2 longitudinal formation measurement.
        #
        # This is independent of the smoothed steering path.
        self.path_gap_pub = self.create_publisher(
            Float64,
            f'/planner/{self.follower_id}/path_gap',
            10,
        )

        self.path_gap_valid_pub = self.create_publisher(
            Bool,
            f'/planner/{self.follower_id}/path_gap_valid',
            10,
        )

        # Latched logical release state for chained followers.
        # R3 can later synchronize its release to R2 without
        # receiving R1's global reference path.
        self.release_state_pub = self.create_publisher(
            Bool,
            f'/planner/{self.follower_id}/released',
            qos,
        )

        self.progress_index_pub = self.create_publisher(
            Int32,
            f'/planner/{self.follower_id}/progress_index',
            10,
        )

        self.progress_acquired_pub = self.create_publisher(
            Bool,
            f'/planner/{self.follower_id}/progress_acquired',
            10,
        )

        self.breadcrumb_count_pub = self.create_publisher(
            Int32,
            f'/planner/{self.follower_id}/breadcrumb_count',
            10,
        )

        self.breadcrumb_pub = self.create_publisher(
            Path,
            self.breadcrumb_topic,
            qos,
        )

        self.parking_pub = self.create_publisher(
            Path,
            self.parking_topic,
            qos,
        )

        self.mode_pub = self.create_publisher(
            String,
            self.mission_state_topic,
            10,
        )

        self.success_pub = self.create_publisher(
            Bool,
            self.mission_success_topic,
            qos,
        )

        self.target_capture_pub = self.create_publisher(
            Bool,
            self.target_capture_topic,
            qos,
        )

        self.reference_distance_pub = self.create_publisher(
            Float64,
            self.reference_distance_topic,
            10,
        )

        # Diagnostic only: R1's lowest historical speed on
        # the breadcrumb trail currently ahead of this follower.
        self.historical_speed_preview_pub = self.create_publisher(
            Float64,
            f'/planner/{self.follower_id}/historical_speed_preview',
            10,
        )

        self.historical_speed_preview_distance_pub = self.create_publisher(
            Float64,
            f'/planner/{self.follower_id}/historical_speed_preview_distance',
            10,
        )

        self.historical_speed_preview_pair_pub = self.create_publisher(
            Float64MultiArray,
            f'/planner/{self.follower_id}/historical_speed_preview_pair',
            10,
        )

        self.historical_speed_recommended_pub = self.create_publisher(
            Float64,
            f'/planner/{self.follower_id}/historical_speed_recommended',
            10,
        )

        self.create_subscription(
            VehicleState,
            self.predecessor_state_topic,
            self.predecessor_callback,
            20,
        )

        self.create_subscription(
            VehicleState,
            self.follower_state_topic,
            self.follower_callback,
            20,
        )

        self.reference_path_subscription = None
        self.predecessor_mission_subscription = None

        if self.terminal_handoff_mode == 'reference_path':
            self.reference_path_subscription = (
                self.create_subscription(
                    Path,
                    self.reference_path_topic,
                    self.reference_path_callback,
                    qos,
                )
            )
        else:
            self.predecessor_mission_subscription = (
                self.create_subscription(
                    String,
                    self.predecessor_mission_state_topic,
                    self.predecessor_mission_state_callback,
                    10,
                )
            )

        if self.release_mode == 'predecessor_release_bootstrap':
            self.create_subscription(
                Bool,
                self.predecessor_release_topic,
                self.predecessor_release_callback,
                qos,
            )

        self.create_subscription(
            Bool,
            self.terminal_settled_topic,
            self.terminal_settled_callback,
            10,
        )

        self.create_subscription(
            Bool,
            self.predecessor_success_topic,
            self.predecessor_success_callback,
            10,
        )

        self.create_subscription(
            Bool,
            self.follower_success_topic,
            self.follower_success_callback,
            10,
        )

        self.create_subscription(
            Float64,
            self.clearance_topic,
            self.clearance_callback,
            20,
        )

        self.create_subscription(
            Bool,
            self.aligned_topic,
            self.aligned_callback,
            20,
        )

        self.timer = self.create_timer(
            0.10,
            self.update,
        )

        self.get_logger().info(
            f'Follower planner ready: 'f'{self.follower_id} <- {self.predecessor_id}'
        )

    # ---------------------------------------------------------
    # Inputs
    # ---------------------------------------------------------

    def append_breadcrumb(self, point, speed):
        # Keep progress valid if the bounded deque ever rolls over.
        if (
            len(self.breadcrumbs)
            == self.breadcrumbs.maxlen
        ):
            self.follower_progress_index = max(
                0,
                self.follower_progress_index - 1,
            )

        self.breadcrumbs.append(point)
        self.breadcrumb_speeds.append(float(speed))

    def predecessor_callback(self, msg):
        self.predecessor = msg

        raw = (
            float(msg.x),
            float(msg.y),
        )

        if self.filtered_predecessor is None:
            self.filtered_predecessor = raw
            self.append_breadcrumb(raw, float(msg.speed))
            return

        if (
            distance(
                raw,
                self.filtered_predecessor,
            )
            > self.jump_limit
        ):
            return

        a = self.filter_alpha

        filtered = (
            a * raw[0]
            + (1.0 - a)
            * self.filtered_predecessor[0],

            a * raw[1]
            + (1.0 - a)
            * self.filtered_predecessor[1],
        )

        self.filtered_predecessor = filtered

        if (
            distance(
                filtered,
                self.breadcrumbs[-1],
            )
            >= self.breadcrumb_spacing
        ):
            self.append_breadcrumb(
                filtered,
                float(msg.speed),
            )

    def follower_callback(self, msg):
        self.follower = msg

    def predecessor_release_callback(self, msg):
        if msg.data:
            self.predecessor_released = True

    def maybe_bootstrap_release(self):
        """
        Bootstrap a later follower from local pair geometry only.

        The synthetic initial breadcrumb line runs from the current
        follower position to the current predecessor position. It
        provides the historical straight segment that physically
        existed before the predecessor began moving.

        No global path, R1 path, or future trajectory is used.
        """
        if (
            self.release_mode
            != 'predecessor_release_bootstrap'
            or self.release_bootstrap_complete
            or not self.predecessor_released
            or self.predecessor is None
            or self.follower is None
        ):
            return

        follower_position = (
            float(self.follower.x),
            float(self.follower.y),
        )

        predecessor_position = (
            float(self.predecessor.x),
            float(self.predecessor.y),
        )

        separation = distance(
            follower_position,
            predecessor_position,
        )

        required = (
            self.breadcrumb_lag_distance
            + self.minimum_extra_trail
        )

        if separation < required:
            return

        spacing = max(
            0.05,
            self.breadcrumb_spacing,
        )

        segment_count = max(
            1,
            int(math.ceil(
                separation / spacing
            )),
        )

        self.breadcrumbs.clear()
        self.breadcrumb_speeds.clear()

        follower_speed = float(
            self.follower.speed
        )

        predecessor_speed = float(
            self.predecessor.speed
        )

        for i in range(segment_count + 1):
            t = i / segment_count

            point = (
                follower_position[0]
                + t * (
                    predecessor_position[0]
                    - follower_position[0]
                ),

                follower_position[1]
                + t * (
                    predecessor_position[1]
                    - follower_position[1]
                ),
            )

            speed = (
                follower_speed
                + t * (
                    predecessor_speed
                    - follower_speed
                )
            )

            self.append_breadcrumb(
                point,
                speed,
            )

        self.filtered_predecessor = (
            predecessor_position
        )

        self.follower_progress_index = 0
        self.progress_acquired = False

        self.trail_ready = True
        self.release_bootstrap_complete = True

        self.get_logger().info(
            f'{self.follower_id} synchronized release '
            f'from {self.predecessor_id}; '
            f'bootstrapped {segment_count + 1} breadcrumbs '
            f'over {separation:.2f} m using pair geometry only.'
        )

        # Publish TRUE immediately rather than waiting for the
        # following 10 Hz planner cycle. This also supports future
        # chained followers such as R4 <- R3.
        self.publish_release_state()

    def clearance_callback(self, msg):
        self.hitbox_clearance = float(
            msg.data
        )

    def aligned_callback(self, msg):
        self.parking_aligned = bool(
            msg.data
        )

    def reference_path_callback(self, msg):
        if self.terminal_handoff_mode != 'reference_path':
            return

        if not msg.poses:
            return

        final = msg.poses[-1].pose.position
        self.final_target = (
            float(final.x),
            float(final.y),
        )

    def predecessor_mission_state_callback(self, msg):
        if self.terminal_handoff_mode != 'predecessor_mission':
            return

        predecessor_mode = str(msg.data)

        if predecessor_mode in (
            self.FORMATION_BRAKE,
            self.FORMATION_HOLD,
        ):
            self.latch_target_capture(
                f'{self.predecessor_id} mission='
                f'{predecessor_mode}'
            )

    def terminal_settled_callback(self, msg):
        self.follower_terminal_settled = bool(
            msg.data
        )

    def latch_target_capture(self, reason):
        if self.target_capture_latched:
            return

        self.target_capture_latched = True
        self.target_capture_candidate_start = None
        self.mode = self.FORMATION_BRAKE
        self.reverse_ready_start = None

        self.get_logger().info(
            f'{self.predecessor_id} terminal handoff latched '
            f'({reason}); {self.follower_id} breadcrumb following '
            'closed and formation braking begins.'
        )

    def predecessor_success_callback(self, msg):
        if msg.data and not self.leader_done:
            self.leader_done = True

            self.get_logger().info(
                f'{self.predecessor_id} SUCCESS latched; terminal '
                f'sequence may continue after {self.follower_id} settles.'
            )

            # Fail-safe only: under normal operation capture occurs first.
            if not self.target_capture_latched:
                self.latch_target_capture(
                    f'{self.predecessor_id} SUCCESS fallback'
                )

    def follower_success_callback(self, msg):
        if msg.data:
            self.follower_done = True

    def publish_target_capture(self):
        msg = Bool()
        msg.data = bool(
            self.target_capture_latched
        )
        self.target_capture_pub.publish(msg)

    def update_target_capture(self, now):
        if self.target_capture_latched:
            return

        if self.predecessor is None or self.final_target is None:
            self.target_capture_candidate_start = None
            return

        error = distance(
            (float(self.predecessor.x), float(self.predecessor.y)),
            self.final_target,
        )

        if error > self.target_capture_tolerance:
            self.target_capture_candidate_start = None
            return

        if self.target_capture_candidate_start is None:
            self.target_capture_candidate_start = now

            if self.target_capture_hold <= 0.0:
                self.latch_target_capture(
                    f'error={error:.3f} m'
                )
            return

        held = (
            now
            - self.target_capture_candidate_start
        ).nanoseconds * 1e-9

        if held >= self.target_capture_hold:
            self.latch_target_capture(
                f'error={error:.3f} m, hold={held:.2f} s'
            )

    # ---------------------------------------------------------
    # Breadcrumbs
    # ---------------------------------------------------------

    def breadcrumb_length(self):
        points = list(self.breadcrumbs)

        return sum(
            distance(a, b)
            for a, b in zip(
                points[:-1],
                points[1:],
            )
        )

    def historical_target(self):
        points = list(self.breadcrumbs)

        if len(points) < 2:
            return None

        remaining = (
            self.breadcrumb_lag_distance
        )

        for i in range(
            len(points) - 1,
            0,
            -1,
        ):
            newer = points[i]
            older = points[i - 1]

            segment = distance(
                newer,
                older,
            )

            if segment < 1e-9:
                continue

            if remaining <= segment:
                ratio = remaining / segment

                target = (
                    newer[0]
                    + ratio
                    * (
                        older[0]
                        - newer[0]
                    ),

                    newer[1]
                    + ratio
                    * (
                        older[1]
                        - newer[1]
                    ),
                )

                return target, i - 1

            remaining -= segment

        return None

    @staticmethod
    def segment_projection(point, a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length2 = dx * dx + dy * dy

        if length2 < 1e-12:
            return 0.0, a, distance(point, a)

        t = (
            (point[0] - a[0]) * dx
            + (point[1] - a[1]) * dy
        ) / length2

        t = max(0.0, min(1.0, t))

        projection = (
            a[0] + t * dx,
            a[1] + t * dy,
        )

        return (
            t,
            projection,
            distance(point, projection),
        )

    def advance_r2_progress(self, history, target_index):
        """
        Track the follower through predecessor breadcrumbs in
        chronological order.

        Startup is special:
        - once enough initial trail exists, perform ONE global
          nearest-segment acquisition;
        - the initial trail is still short and contains no
          self-near stress-course loops.

        After acquisition:
        - never search arbitrary future segments again;
        - consume only the current chronological segment;
        - advance when its endpoint is captured or its
          perpendicular plane is crossed inside the corridor.
        """
        if self.follower is None or len(history) < 2:
            return

        max_segment = min(
            target_index,
            len(history) - 2,
        )

        if max_segment < 0:
            return

        follower_position = (
            float(self.follower.x),
            float(self.follower.y),
        )

        # ----------------------------------------------------
        # ONE-TIME acquisition.
        #
        # Do not acquire until the same amount of trail needed
        # to release the follower already exists. At ~11.4 m,
        # this is still on the initial straight and therefore
        # cannot contain a later self-near loop.
        # ----------------------------------------------------

        if not self.progress_acquired:
            required = (
                self.breadcrumb_lag_distance
                + self.minimum_extra_trail
            )

            trail_length = sum(
                distance(a, b)
                for a, b in zip(
                    history[:-1],
                    history[1:],
                )
            )

            if trail_length < required:
                return

            best_index = 0
            best_distance = float('inf')

            for i in range(max_segment + 1):
                _, _, d = self.segment_projection(
                    follower_position,
                    history[i],
                    history[i + 1],
                )

                if d < best_distance:
                    best_distance = d
                    best_index = i

            self.follower_progress_index = best_index
            self.progress_acquired = True

            self.get_logger().info(
                f'{self.follower_id} breadcrumb progress acquired '
                f'at segment {best_index}; '
                f'distance={best_distance:.2f} m.'
            )

            return

        # ----------------------------------------------------
        # LOCKED chronological progression.
        # ----------------------------------------------------

        if self.follower_progress_index > max_segment:
            return

        advances = 0

        max_advances = max(
            1,
            int(round(
                self.max_progress_advances_per_cycle
            )),
        )

        while (
            self.follower_progress_index < max_segment
            and advances < max_advances
        ):
            i = self.follower_progress_index

            a = history[i]
            b = history[i + 1]

            dx = b[0] - a[0]
            dy = b[1] - a[1]

            segment_length = math.hypot(dx, dy)

            if segment_length < 1e-9:
                self.follower_progress_index += 1
                advances += 1
                continue

            ux = dx / segment_length
            uy = dy / segment_length

            rel_x = follower_position[0] - a[0]
            rel_y = follower_position[1] - a[1]

            along = (
                rel_x * ux
                + rel_y * uy
            )

            lateral = abs(
                rel_x * (-uy)
                + rel_y * ux
            )

            endpoint_distance = distance(
                follower_position,
                b,
            )

            endpoint_captured = (
                endpoint_distance
                <= self.progress_capture_radius
            )

            segment_crossed = (
                along
                >= segment_length
                + self.passed_point_margin
                and lateral
                <= self.progress_corridor_m
            )

            if endpoint_captured or segment_crossed:
                self.follower_progress_index += 1
                advances += 1
                continue

            break

    def update_follower_progress(self):
        if self.follower is None:
            return

        history = list(self.breadcrumbs)

        if len(history) < 2:
            return

        self.advance_r2_progress(
            history,
            len(history) - 2,
        )

    def publish_release_state(self):
        msg = Bool()
        msg.data = bool(self.trail_ready)
        self.release_state_pub.publish(msg)

    def publish_progress_diagnostics(self):
        index = Int32()
        index.data = int(
            self.follower_progress_index
        )
        self.progress_index_pub.publish(index)

        acquired = Bool()
        acquired.data = bool(
            self.progress_acquired
        )
        self.progress_acquired_pub.publish(acquired)

        count = Int32()
        count.data = int(
            len(self.breadcrumbs)
        )
        self.breadcrumb_count_pub.publish(count)

    def path_gap_measurement(self):
        """
        V2.2 longitudinal formation measurement.

        IMPORTANT:
        This uses the UNSMOOTHED historical breadcrumb chain.

        Guidance may smooth/interpolate that chain later, but smoothing
        must never affect the longitudinal distance PID.

        Returns:
            bumper_gap_m

        where:
            reference_separation
                = leader progress - follower progress

            bumper_gap
                = reference_separation
                  - predecessor rear extent
                  - follower front extent
        """
        if (
            self.predecessor is None
            or self.follower is None
        ):
            return None

        history = list(self.breadcrumbs)

        if len(history) < 2:
            return None

        # Progress is advanced exactly once per planner cycle
        # in update(). Measurements only consume the locked index.

        # Cumulative arc length of the ORIGINAL breadcrumb chain.
        cumulative = [0.0]

        for a, b in zip(
            history[:-1],
            history[1:],
        ):
            cumulative.append(
                cumulative[-1] + distance(a, b)
            )

        if cumulative[-1] <= 1e-9:
            return None

        # ----------------------------------------------------
        # R2 continuous progress.
        #
        # follower_progress_index identifies the correct raw
        # breadcrumb segment. Projection gives sub-segment
        # resolution instead of quantizing to 0.20 m.
        # ----------------------------------------------------

        i = min(
            self.follower_progress_index,
            len(history) - 2,
        )

        r2_position = (
            float(self.follower.x),
            float(self.follower.y),
        )

        t, _, _ = self.segment_projection(
            r2_position,
            history[i],
            history[i + 1],
        )

        segment_length = distance(
            history[i],
            history[i + 1],
        )

        follower_s = (
            cumulative[i]
            + t * segment_length
        )

        # ----------------------------------------------------
        # R1 progress.
        #
        # The last stored breadcrumb can lag the current
        # filtered R1 location by < breadcrumb_spacing.
        # Add that small unsaved tail continuously.
        # ----------------------------------------------------

        leader_s = cumulative[-1]

        if self.filtered_predecessor is not None:
            tail = distance(
                history[-1],
                self.filtered_predecessor,
            )

            if tail <= self.jump_limit:
                leader_s += tail

        reference_separation = (
            leader_s - follower_s
        )

        bumper_gap = (
            reference_separation
            - self.predecessor_rear_extent
            - self.follower_front_extent
        )

        return float(bumper_gap)

    def publish_path_gap(self):
        measurement = self.path_gap_measurement()

        valid = Bool()
        valid.data = measurement is not None
        self.path_gap_valid_pub.publish(valid)

        if measurement is None:
            return

        msg = Float64()
        msg.data = measurement
        self.path_gap_pub.publish(msg)

    def smooth_curve(self, points):
        """
        Interpolating cubic Hermite / Catmull-Rom-style curve.

        The curve passes through the breadcrumbs, but heading is
        continuous instead of pointing from waypoint to waypoint.
        """
        if len(points) <= 2:
            return points

        samples = max(
            1,
            int(round(self.curve_samples_per_segment)),
        )

        tangent_scale = self.curve_tangent_scale

        tangents = []

        for i in range(len(points)):
            if i == 0:
                tx = points[1][0] - points[0][0]
                ty = points[1][1] - points[0][1]
            elif i == len(points) - 1:
                tx = points[-1][0] - points[-2][0]
                ty = points[-1][1] - points[-2][1]
            else:
                tx = 0.5 * (
                    points[i + 1][0]
                    - points[i - 1][0]
                )
                ty = 0.5 * (
                    points[i + 1][1]
                    - points[i - 1][1]
                )

            tangents.append((
                tangent_scale * tx,
                tangent_scale * ty,
            ))

        curve = [points[0]]

        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]
            m0 = tangents[i]
            m1 = tangents[i + 1]

            for step in range(1, samples + 1):
                t = step / samples
                t2 = t * t
                t3 = t2 * t

                h00 = 2.0 * t3 - 3.0 * t2 + 1.0
                h10 = t3 - 2.0 * t2 + t
                h01 = -2.0 * t3 + 3.0 * t2
                h11 = t3 - t2

                point = (
                    h00 * p0[0]
                    + h10 * m0[0]
                    + h01 * p1[0]
                    + h11 * m1[0],

                    h00 * p0[1]
                    + h10 * m0[1]
                    + h01 * p1[1]
                    + h11 * m1[1],
                )

                if distance(curve[-1], point) >= 0.01:
                    curve.append(point)

        return curve

    def build_breadcrumb_path(self):
        """
        Publish the leader's smooth historical centerline ahead of R2.

        Formation spacing is NOT encoded by choosing one breadcrumb as
        a steering target. The 5 m hull gap is handled independently by
        the follower's speed controller. This prevents an overshoot from
        ever creating a "turn around and recover an old waypoint" command.
        """
        if self.follower is None:
            return None

        history = list(self.breadcrumbs)

        if len(history) < 2:
            return None

        # Progress is advanced exactly once per planner cycle
        # in update(). Path construction only consumes that index.

        # Include a few already-passed points only so the interpolated
        # curve has a stable tangent immediately behind the projection.
        # Steering is based on curve tangent, never point chasing.
        start_index = max(
            0,
            self.follower_progress_index
            - int(round(self.curve_backtrack_points)),
        )

        control_points = history[start_index:]

        if len(control_points) < 2:
            return None

        return self.smooth_curve(
            control_points
        )


    def historical_speed_preview(self):
        """
        Return:
            minimum predecessor speed still ahead,
            approximate trail distance to that speed point.

        Diagnostic only -- no control effect.
        """
        points = list(self.breadcrumbs)
        speeds = list(self.breadcrumb_speeds)

        count = min(
            len(points),
            len(speeds),
        )

        if count < 2:
            return None

        start = max(
            0,
            min(
                self.follower_progress_index,
                count - 1,
            ),
        )

        slow_index = min(
            range(start, count),
            key=lambda i: speeds[i],
        )

        slow_speed = speeds[slow_index]

        distance_ahead = 0.0

        for i in range(start, slow_index):
            distance_ahead += distance(
                points[i],
                points[i + 1],
            )

        return (
            float(slow_speed),
            float(distance_ahead),
        )

    # ---------------------------------------------------------
    # Parking
    # ---------------------------------------------------------

    def terminal_heading_vector(self):
        points = list(self.breadcrumbs)

        if len(points) >= 8:
            older = points[-8]
            newer = points[-1]

            dx = newer[0] - older[0]
            dy = newer[1] - older[1]

            length = math.hypot(dx, dy)

            if length > 0.1:
                return (
                    dx / length,
                    dy / length,
                )

        return (
            math.cos(self.predecessor.course_angle),
            math.sin(self.predecessor.course_angle),
        )

    def line_path(self, start, goal):
        length = distance(start, goal)

        count = max(
            1,
            int(
                math.ceil(
                    length
                    / self.parking_spacing
                )
            ),
        )

        return [
            (
                start[0]
                + (
                    goal[0] - start[0]
                )
                * i / count,

                start[1]
                + (
                    goal[1] - start[1]
                )
                * i / count,
            )
            for i in range(count + 1)
        ]

    def build_parking_path(self):
        r1 = (
            float(self.predecessor.x),
            float(self.predecessor.y),
        )

        r2 = (
            float(self.follower.x),
            float(self.follower.y),
        )

        hx, hy = self.terminal_heading_vector()

        px = -hy
        py = hx

        candidate_a = (
            r1[0]
            + self.parking_separation * px,
            r1[1]
            + self.parking_separation * py,
        )

        candidate_b = (
            r1[0]
            - self.parking_separation * px,
            r1[1]
            - self.parking_separation * py,
        )

        if (
            distance(r2, candidate_a)
            <= distance(r2, candidate_b)
        ):
            final = candidate_a
            sx, sy = px, py
        else:
            final = candidate_b
            sx, sy = -px, -py

        # First parking leg moves sideways AWAY from the
        # leader's final trajectory before heading to final.
        escape = (
            r2[0]
            + self.parking_escape_distance * sx,
            r2[1]
            + self.parking_escape_distance * sy,
        )

        leg1 = self.line_path(
            r2,
            escape,
        )

        leg2 = self.line_path(
            escape,
            final,
        )

        self.get_logger().info(
            f'R2 parking target created; '
            f'reference separation = '
            f'{distance(r1, final):.2f} m.'
        )

        return leg1 + leg2[1:]

    @staticmethod
    def path_message(points):
        msg = Path()
        msg.header.frame_id = 'world_ned'

        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = 'world_ned'
            pose.pose.position.x = float(
                point[0]
            )
            pose.pose.position.y = float(
                point[1]
            )
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        return msg

    # ---------------------------------------------------------
    # Mission
    # ---------------------------------------------------------

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

        if self.mode != self.previous_mode:
            self.get_logger().info(
                f'Mission state -> {self.mode}'
            )
            self.previous_mode = self.mode

    def publish_reference_distance(self):
        if self.predecessor is None or self.follower is None:
            return

        msg = Float64()
        msg.data = math.hypot(
            self.predecessor.x - self.follower.x,
            self.predecessor.y - self.follower.y,
        )
        self.reference_distance_pub.publish(
            msg
        )

    def update(self):
        now = self.get_clock().now()

        # First follower (R2 <- R1) detects terminal capture from
        # the global reference endpoint. Later followers receive the
        # handoff only from their predecessor's mission state.
        if self.terminal_handoff_mode == 'reference_path':
            self.update_target_capture(now)

        self.publish_target_capture()
        self.publish_mode()
        self.publish_reference_distance()

        # Later followers may initialize their local historical
        # breadcrumb chain from predecessor/follower geometry when
        # the predecessor is released.
        self.maybe_bootstrap_release()

        # Single authoritative chronological progress update.
        self.update_follower_progress()

        # V2.2: unsmoothed breadcrumb arc-length formation gap.
        self.publish_path_gap()
        self.publish_progress_diagnostics()
        self.publish_release_state()

        if self.predecessor is None or self.follower is None:
            return

        if self.mode == self.FOLLOW:
            required = (
                self.breadcrumb_lag_distance
                + self.minimum_extra_trail
            )

            trail = self.breadcrumb_length()

            if trail < required:
                return

            if not self.trail_ready:
                self.trail_ready = True

                self.get_logger().info(
                    f'{self.predecessor_id} trail ready: '
                    f'{trail:.2f} m; {self.follower_id} released.'
                )

                self.publish_release_state()

            path = self.build_breadcrumb_path()

            if path:
                preview = self.historical_speed_preview()

                if preview is not None:
                    preview_speed, preview_distance = preview

                    msg = Float64()
                    msg.data = float(preview_speed)
                    self.historical_speed_preview_pub.publish(msg)

                    msg = Float64()
                    msg.data = float(preview_distance)
                    self.historical_speed_preview_distance_pub.publish(msg)

                    recommended_speed = math.sqrt(
                        max(
                            0.0,
                            preview_speed * preview_speed
                            + 2.0
                            * self.historical_preview_decel
                            * preview_distance,
                        )
                    )

                    recommended_speed = min(
                        recommended_speed,
                        self.historical_preview_max_speed,
                    )

                    pair = Float64MultiArray()
                    pair.data = [
                        float(preview_speed),
                        float(preview_distance),
                        float(recommended_speed),
                    ]
                    self.historical_speed_preview_pair_pub.publish(pair)

                    msg = Float64()
                    msg.data = float(recommended_speed)
                    self.historical_speed_recommended_pub.publish(msg)

                self.breadcrumb_pub.publish(
                    self.path_message(path)
                )

        elif self.mode == self.FORMATION_BRAKE:
            # Controller performs BRAKE -> RECOVER -> HOLD while
            # preserving the last valid trail tangent heading.
            if self.follower_terminal_settled:
                self.mode = self.FORMATION_HOLD

        elif self.mode == self.FORMATION_HOLD:
            # Terminal progression is forbidden until BOTH robots
            # in this predecessor -> follower pair are ready.
            if not (
                self.leader_done
                and self.follower_terminal_settled
            ):
                return

            # Three-robot platooning mode: remain in active
            # formation hold. Publish pair success so a downstream
            # follower can complete its own terminal sequence, but
            # do not reverse, park, or leave formation.
            if self.terminal_behavior == 'hold':
                msg = Bool()
                msg.data = True
                self.success_pub.publish(msg)
                return

            if (
                self.hitbox_clearance
                < self.parking_reverse_clearance
            ):
                self.mode = self.PARK_REVERSE
                self.reverse_ready_start = None
                return

            # The formation already stopped with enough hull gap,
            # so reverse is unnecessary.
            self.parking_path = (
                self.build_parking_path()
            )
            self.parking_aligned = False
            self.parking_pub.publish(
                self.path_message(
                    self.parking_path
                )
            )
            self.mode = self.PARK_ALIGN

        elif self.mode == self.PARK_REVERSE:
            if (
                self.hitbox_clearance
                < self.parking_reverse_clearance
            ):
                self.reverse_ready_start = None
                return

            if self.reverse_ready_start is None:
                self.reverse_ready_start = now
                return

            held = (
                now
                - self.reverse_ready_start
            ).nanoseconds * 1e-9

            if held >= self.parking_reverse_hold:
                self.parking_path = (
                    self.build_parking_path()
                )
                self.parking_aligned = False
                self.parking_pub.publish(
                    self.path_message(
                        self.parking_path
                    )
                )
                self.mode = self.PARK_ALIGN

        elif self.mode == self.PARK_ALIGN:
            self.parking_pub.publish(
                self.path_message(
                    self.parking_path
                )
            )

            if self.parking_aligned:
                self.mode = self.PARK
                self.follower_done = False

        elif self.mode == self.PARK:
            self.parking_pub.publish(
                self.path_message(
                    self.parking_path
                )
            )

            if self.follower_done:
                self.mode = self.DWELL
                self.dwell_start = now

        elif self.mode == self.DWELL:
            held = (
                now
                - self.dwell_start
            ).nanoseconds * 1e-9

            if held >= self.dwell_time:
                self.mode = self.SUCCESS

        elif self.mode == self.SUCCESS:
            msg = Bool()
            msg.data = True
            self.success_pub.publish(msg)


def main():
    rclpy.init()

    node = FollowerPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
