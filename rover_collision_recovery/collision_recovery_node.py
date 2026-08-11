#!/usr/bin/env python3

from enum import Enum, auto
import math
from typing import Optional

import rclpy

from geometry_msgs.msg import Twist, Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class RecoveryState(Enum):
    NORMAL = auto()
    PAUSE = auto()
    ESCAPING = auto()
    COOLDOWN = auto()


class CollisionZone(Enum):
    NONE = auto()

    FRONT = auto()
    REAR = auto()

    LEFT_FRONT = auto()
    LEFT_REAR = auto()

    RIGHT_FRONT = auto()
    RIGHT_REAR = auto()


class EscapeMode(Enum):
    NONE = auto()

    FORWARD = auto()
    BACKWARD = auto()

    FORWARD_LEFT = auto()
    FORWARD_RIGHT = auto()

    BACKWARD_LEFT = auto()
    BACKWARD_RIGHT = auto()

    TURN_LEFT = auto()
    TURN_RIGHT = auto()


class CollisionRecoveryNode(Node):

    def __init__(self):
        super().__init__('collision_recovery')

        # ============================================================
        # TOPICS
        # ============================================================

        self.declare_parameter(
            'cmd_vel_in_topic',
            '/cmd_vel_smoothed'
        )

        self.declare_parameter(
            'cmd_vel_out_topic',
            '/cmd_vel_recovery'
        )

        self.declare_parameter(
            'scan_topic',
            '/scan'
        )

        # ============================================================
        # ZONES NORMALES
        # VERSION DEBUT DE JOURNEE
        # ============================================================

        self.declare_zone(
            'front',
            0.59,
            0.90,
            -0.40,
            0.40
        )

        self.declare_zone(
            'rear',
            -0.95,
            -0.59,
            -0.40,
            0.40
        )

        self.declare_zone(
            'left_front',
            0.00,
            0.59,
            0.40,
            0.50
        )

        self.declare_zone(
            'left_rear',
            -0.59,
            0.00,
            0.40,
            0.50
        )

        self.declare_zone(
            'right_front',
            0.00,
            0.59,
            -0.50,
            -0.40
        )

        self.declare_zone(
            'right_rear',
            -0.59,
            0.00,
            -0.50,
            -0.40
        )

        # ============================================================
        # ZONES RECOVERY ELARGIES
        # VERSION DEBUT DE JOURNEE
        # ============================================================

        self.declare_zone(
            'recovery_front',
            0.50,
            1.05,
            -0.45,
            0.45
        )

        self.declare_zone(
            'recovery_rear',
            -1.10,
            -0.50,
            -0.45,
            0.45
        )

        self.declare_zone(
            'recovery_left_front',
            -0.10,
            0.70,
            0.35,
            0.60
        )

        self.declare_zone(
            'recovery_left_rear',
            -0.70,
            0.10,
            0.35,
            0.60
        )

        self.declare_zone(
            'recovery_right_front',
            -0.10,
            0.70,
            -0.60,
            -0.35
        )

        self.declare_zone(
            'recovery_right_rear',
            -0.70,
            0.10,
            -0.60,
            -0.35
        )

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter(
            'trigger_min_points',
            3
        )

        self.declare_parameter(
            'escape_stop_min_points',
            3
        )

        self.declare_parameter(
            'recovery_clear_max_points',
            0
        )

        self.declare_parameter(
            'minimum_linear_command',
            0.03
        )

        self.declare_parameter(
            'minimum_angular_command',
            0.10
        )

        self.declare_parameter(
            'pause_duration',
            0.25
        )

        self.declare_parameter(
            'forward_escape_speed',
            0.20
        )

        self.declare_parameter(
            'backward_escape_speed',
            -0.20
        )

        self.declare_parameter(
            'turn_escape_speed',
            0.20
        )

        self.declare_parameter(
            'max_linear_escape_duration',
            8.0
        )

        self.declare_parameter(
            'max_turn_escape_duration',
            6.0
        )

        self.declare_parameter(
            'clear_hold_duration',
            0.50
        )

        self.declare_parameter(
            'cooldown_duration',
            0.50
        )

        self.declare_parameter(
            'max_direction_switches',
            1
        )

        self.declare_parameter(
            'scan_timeout',
            0.50
        )

        self.declare_parameter(
            'command_timeout',
            0.50
        )

        self.declare_parameter(
            'control_frequency',
            20.0
        )

        # ============================================================
        # ROS
        # ============================================================

        cmd_in = str(
            self.get_parameter(
                'cmd_vel_in_topic'
            ).value
        )

        cmd_out = str(
            self.get_parameter(
                'cmd_vel_out_topic'
            ).value
        )

        scan_topic = str(
            self.get_parameter(
                'scan_topic'
            ).value
        )

        # Commande vers Collision Monitor
        self.publisher = self.create_publisher(
            Twist,
            cmd_out,
            10
        )

        # ============================================================
        # RVIZ MARKERS
        # ============================================================

        self.zone_marker_publisher = self.create_publisher(
            MarkerArray,
            '/collision_recovery/zones',
            10
        )

        # ============================================================
        # SUBSCRIPTIONS
        # ============================================================

        self.create_subscription(
            Twist,
            cmd_in,
            self.cmd_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        frequency = max(
            1.0,
            float(
                self.get_parameter(
                    'control_frequency'
                ).value
            )
        )

        # Main recovery control loop
        self.timer = self.create_timer(
            1.0 / frequency,
            self.control_loop
        )

        # RViz visualization at 2 Hz
        self.marker_timer = self.create_timer(
            0.5,
            self.publish_zone_markers
        )

        # ============================================================
        # STATE
        # ============================================================

        self.latest_cmd = Twist()
        self.latest_scan: Optional[LaserScan] = None

        self.latest_cmd_time = None
        self.latest_scan_time = None

        self.state = RecoveryState.NORMAL

        self.escape_mode = EscapeMode.NONE
        self.trigger_zone = CollisionZone.NONE

        self.state_start_time = (
            self.get_clock().now()
        )

        self.clear_since = None
        self.direction_switch_count = 0

        self.get_logger().info(
            f'6-zone collision recovery ready: '
            f'{cmd_in} -> {cmd_out}'
        )

        self.get_logger().info(
            'RViz recovery zones published on '
            '/collision_recovery/zones'
        )

    # ================================================================
    # ZONES
    # ================================================================

    def declare_zone(
        self,
        name,
        x_min,
        x_max,
        y_min,
        y_max
    ):

        self.declare_parameter(
            f'{name}_x_min',
            x_min
        )

        self.declare_parameter(
            f'{name}_x_max',
            x_max
        )

        self.declare_parameter(
            f'{name}_y_min',
            y_min
        )

        self.declare_parameter(
            f'{name}_y_max',
            y_max
        )

    def get_zone(self, name):

        return (
            float(
                self.get_parameter(
                    f'{name}_x_min'
                ).value
            ),
            float(
                self.get_parameter(
                    f'{name}_x_max'
                ).value
            ),
            float(
                self.get_parameter(
                    f'{name}_y_min'
                ).value
            ),
            float(
                self.get_parameter(
                    f'{name}_y_max'
                ).value
            )
        )

    @staticmethod
    def point_in_zone(
        x,
        y,
        rect
    ):

        x_min, x_max, y_min, y_max = rect

        return (
            x_min <= x <= x_max
            and
            y_min <= y <= y_max
        )

    # ================================================================
    # CLASSIFICATION
    # ================================================================

    def classify_point(
        self,
        x,
        y,
        recovery=False
    ):

        prefix = (
            'recovery_'
            if recovery
            else ''
        )

        zones = [
            (
                CollisionZone.FRONT,
                f'{prefix}front'
            ),
            (
                CollisionZone.REAR,
                f'{prefix}rear'
            ),
            (
                CollisionZone.LEFT_FRONT,
                f'{prefix}left_front'
            ),
            (
                CollisionZone.LEFT_REAR,
                f'{prefix}left_rear'
            ),
            (
                CollisionZone.RIGHT_FRONT,
                f'{prefix}right_front'
            ),
            (
                CollisionZone.RIGHT_REAR,
                f'{prefix}right_rear'
            ),
        ]

        for zone, parameter_name in zones:

            rect = self.get_zone(
                parameter_name
            )

            if self.point_in_zone(
                x,
                y,
                rect
            ):
                return zone

        return CollisionZone.NONE

    # ================================================================
    # RVIZ VISUALIZATION
    # ================================================================

    def create_rectangle_marker(
        self,
        marker_id,
        namespace,
        rect,
        z,
        r,
        g,
        b,
        line_width=0.025
    ):

        x_min, x_max, y_min, y_max = rect

        marker = Marker()

        marker.header.frame_id = 'base_link'
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = line_width

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        coordinates = [
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
            (x_min, y_min),
        ]

        for x, y in coordinates:

            point = Point()

            point.x = x
            point.y = y
            point.z = z

            marker.points.append(
                point
            )

        return marker

    def create_text_marker(
        self,
        marker_id,
        namespace,
        text,
        rect,
        z,
        r,
        g,
        b
    ):

        x_min, x_max, y_min, y_max = rect

        marker = Marker()

        marker.header.frame_id = 'base_link'
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = (
            x_min + x_max
        ) / 2.0

        marker.pose.position.y = (
            y_min + y_max
        ) / 2.0

        marker.pose.position.z = z

        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.08

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        marker.text = text

        return marker

    def publish_zone_markers(self):

        marker_array = MarkerArray()

        marker_id = 0

        # ============================================================
        # ZONES NORMALES
        #
        # VERT
        # ============================================================

        normal_zones = [
            (
                'FRONT',
                'front'
            ),
            (
                'REAR',
                'rear'
            ),
            (
                'LEFT_FRONT',
                'left_front'
            ),
            (
                'LEFT_REAR',
                'left_rear'
            ),
            (
                'RIGHT_FRONT',
                'right_front'
            ),
            (
                'RIGHT_REAR',
                'right_rear'
            ),
        ]

        for label, parameter_name in normal_zones:

            rect = self.get_zone(
                parameter_name
            )

            marker_array.markers.append(
                self.create_rectangle_marker(
                    marker_id,
                    'normal_zones',
                    rect,
                    0.03,
                    0.0,
                    1.0,
                    0.0,
                    line_width=0.020
                )
            )

            marker_id += 1

            marker_array.markers.append(
                self.create_text_marker(
                    marker_id,
                    'normal_labels',
                    label,
                    rect,
                    0.08,
                    0.0,
                    1.0,
                    0.0
                )
            )

            marker_id += 1

        # ============================================================
        # ZONES RECOVERY
        #
        # ROUGE
        # ============================================================

        recovery_zones = [
            (
                'RECOVERY_FRONT',
                'recovery_front'
            ),
            (
                'RECOVERY_REAR',
                'recovery_rear'
            ),
            (
                'RECOVERY_LEFT_FRONT',
                'recovery_left_front'
            ),
            (
                'RECOVERY_LEFT_REAR',
                'recovery_left_rear'
            ),
            (
                'RECOVERY_RIGHT_FRONT',
                'recovery_right_front'
            ),
            (
                'RECOVERY_RIGHT_REAR',
                'recovery_right_rear'
            ),
        ]

        for label, parameter_name in recovery_zones:

            rect = self.get_zone(
                parameter_name
            )

            marker_array.markers.append(
                self.create_rectangle_marker(
                    marker_id,
                    'recovery_zones',
                    rect,
                    0.06,
                    1.0,
                    0.0,
                    0.0,
                    line_width=0.035
                )
            )

            marker_id += 1

            marker_array.markers.append(
                self.create_text_marker(
                    marker_id,
                    'recovery_labels',
                    label,
                    rect,
                    0.13,
                    1.0,
                    0.0,
                    0.0
                )
            )

            marker_id += 1

        self.zone_marker_publisher.publish(
            marker_array
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def cmd_callback(self, msg):

        self.latest_cmd = msg

        self.latest_cmd_time = (
            self.get_clock().now()
        )

    def scan_callback(self, msg):

        self.latest_scan = msg

        self.latest_scan_time = (
            self.get_clock().now()
        )

    # ================================================================
    # TIME
    # ================================================================

    def elapsed(self):

        return (
            self.get_clock().now()
            - self.state_start_time
        ).nanoseconds / 1e9

    def is_fresh(
        self,
        stamp,
        timeout_name
    ):

        if stamp is None:
            return False

        timeout = float(
            self.get_parameter(
                timeout_name
            ).value
        )

        age = (
            self.get_clock().now()
            - stamp
        ).nanoseconds / 1e9

        return age <= timeout

    def set_state(
        self,
        state,
        reason
    ):

        if state == self.state:
            return

        self.state = state

        self.state_start_time = (
            self.get_clock().now()
        )

        self.clear_since = None

        self.get_logger().warn(
            f'{state.name}: {reason}'
        )

    # ================================================================
    # SCAN
    # ================================================================

    def empty_counts(self):

        return {
            CollisionZone.FRONT: 0,
            CollisionZone.REAR: 0,

            CollisionZone.LEFT_FRONT: 0,
            CollisionZone.LEFT_REAR: 0,

            CollisionZone.RIGHT_FRONT: 0,
            CollisionZone.RIGHT_REAR: 0,
        }

    def count_zone_points(self):

        normal = self.empty_counts()
        recovery = self.empty_counts()

        if self.latest_scan is None:
            return normal, recovery

        angle = self.latest_scan.angle_min

        for distance in self.latest_scan.ranges:

            if (
                math.isfinite(distance)
                and
                self.latest_scan.range_min
                <= distance
                <= self.latest_scan.range_max
            ):

                x = (
                    distance
                    * math.cos(angle)
                )

                y = (
                    distance
                    * math.sin(angle)
                )

                normal_zone = self.classify_point(
                    x,
                    y,
                    recovery=False
                )

                if (
                    normal_zone
                    != CollisionZone.NONE
                ):

                    normal[
                        normal_zone
                    ] += 1

                recovery_zone = self.classify_point(
                    x,
                    y,
                    recovery=True
                )

                if (
                    recovery_zone
                    != CollisionZone.NONE
                ):

                    recovery[
                        recovery_zone
                    ] += 1

            angle += (
                self.latest_scan.angle_increment
            )

        return normal, recovery

    # ================================================================
    # COMMANDS
    # ================================================================

    @staticmethod
    def stop_command():

        return Twist()

    def escape_command(self):

        command = Twist()

        forward = float(
            self.get_parameter(
                'forward_escape_speed'
            ).value
        )

        backward = float(
            self.get_parameter(
                'backward_escape_speed'
            ).value
        )

        turn = abs(
            float(
                self.get_parameter(
                    'turn_escape_speed'
                ).value
            )
        )

        if (
            self.escape_mode
            == EscapeMode.FORWARD
        ):

            command.linear.x = forward

        elif (
            self.escape_mode
            == EscapeMode.BACKWARD
        ):

            command.linear.x = backward

        elif (
            self.escape_mode
            == EscapeMode.FORWARD_LEFT
        ):

            command.linear.x = forward
            command.angular.z = turn

        elif (
            self.escape_mode
            == EscapeMode.FORWARD_RIGHT
        ):

            command.linear.x = forward
            command.angular.z = -turn

        elif (
            self.escape_mode
            == EscapeMode.BACKWARD_LEFT
        ):

            command.linear.x = backward
            command.angular.z = turn

        elif (
            self.escape_mode
            == EscapeMode.BACKWARD_RIGHT
        ):

            command.linear.x = backward
            command.angular.z = -turn

        elif (
            self.escape_mode
            == EscapeMode.TURN_LEFT
        ):

            command.angular.z = turn

        elif (
            self.escape_mode
            == EscapeMode.TURN_RIGHT
        ):

            command.angular.z = -turn

        return command

    # ================================================================
    # ESCAPE TYPE
    # ================================================================

    def escape_is_forward(self):

        return self.escape_mode in (
            EscapeMode.FORWARD,
            EscapeMode.FORWARD_LEFT,
            EscapeMode.FORWARD_RIGHT
        )

    def escape_is_backward(self):

        return self.escape_mode in (
            EscapeMode.BACKWARD,
            EscapeMode.BACKWARD_LEFT,
            EscapeMode.BACKWARD_RIGHT
        )

    def escape_is_linear(self):

        return (
            self.escape_is_forward()
            or
            self.escape_is_backward()
        )

    def max_escape_duration(self):

        if self.escape_is_linear():

            return float(
                self.get_parameter(
                    'max_linear_escape_duration'
                ).value
            )

        return float(
            self.get_parameter(
                'max_turn_escape_duration'
            ).value
        )

    # ================================================================
    # BLOCKED DIRECTIONS
    # ================================================================

    def front_blocked(self, counts):

        threshold = int(
            self.get_parameter(
                'escape_stop_min_points'
            ).value
        )

        return (
            counts[
                CollisionZone.FRONT
            ] >= threshold

            or

            counts[
                CollisionZone.LEFT_FRONT
            ] >= threshold

            or

            counts[
                CollisionZone.RIGHT_FRONT
            ] >= threshold
        )

    def rear_blocked(self, counts):

        threshold = int(
            self.get_parameter(
                'escape_stop_min_points'
            ).value
        )

        return (
            counts[
                CollisionZone.REAR
            ] >= threshold

            or

            counts[
                CollisionZone.LEFT_REAR
            ] >= threshold

            or

            counts[
                CollisionZone.RIGHT_REAR
            ] >= threshold
        )

    # ================================================================
    # CHOOSE ZONE
    # ================================================================

    def choose_zone(
        self,
        counts,
        allowed_zones
    ):

        threshold = int(
            self.get_parameter(
                'trigger_min_points'
            ).value
        )

        valid = [
            zone
            for zone in allowed_zones
            if counts[zone] >= threshold
        ]

        if not valid:
            return CollisionZone.NONE

        return max(
            valid,
            key=lambda zone: counts[zone]
        )

    # ================================================================
    # ESCAPE FOR COLLISION
    # VERSION DIRECTE DU DEBUT
    # ================================================================

    def escape_for_zone(
        self,
        zone
    ):

        if (
            zone
            == CollisionZone.FRONT
        ):

            return EscapeMode.BACKWARD

        if (
            zone
            == CollisionZone.REAR
        ):

            return EscapeMode.FORWARD

        if (
            zone
            == CollisionZone.LEFT_FRONT
        ):

            return EscapeMode.BACKWARD_RIGHT

        if (
            zone
            == CollisionZone.RIGHT_FRONT
        ):

            return EscapeMode.BACKWARD_LEFT

        if (
            zone
            == CollisionZone.LEFT_REAR
        ):

            return EscapeMode.FORWARD_RIGHT

        if (
            zone
            == CollisionZone.RIGHT_REAR
        ):

            return EscapeMode.FORWARD_LEFT

        return EscapeMode.NONE

    # ================================================================
    # RECOVERY CLEARANCE
    # ================================================================

    def recovery_zone_clear(
        self,
        recovery
    ):

        if (
            self.trigger_zone
            == CollisionZone.NONE
        ):

            return False

        points = recovery[
            self.trigger_zone
        ]

        max_points = int(
            self.get_parameter(
                'recovery_clear_max_points'
            ).value
        )

        if points > max_points:

            self.clear_since = None

            return False

        now = (
            self.get_clock().now()
        )

        if self.clear_since is None:

            self.clear_since = now

            self.get_logger().info(
                f'{self.trigger_zone.name} '
                f'recovery zone clear '
                f'({points} points)'
            )

            return False

        hold = float(
            self.get_parameter(
                'clear_hold_duration'
            ).value
        )

        clear_elapsed = (
            now
            - self.clear_since
        ).nanoseconds / 1e9

        return (
            clear_elapsed
            >= hold
        )

    # ================================================================
    # STATE CHANGES
    # ================================================================

    def start_recovery(
        self,
        mode,
        zone,
        reason
    ):

        self.direction_switch_count = 0

        self.escape_mode = mode
        self.trigger_zone = zone

        self.clear_since = None

        self.publisher.publish(
            self.stop_command()
        )

        self.set_state(
            RecoveryState.PAUSE,
            reason
        )

    def finish_recovery(
        self,
        reason
    ):

        self.publisher.publish(
            self.stop_command()
        )

        self.escape_mode = (
            EscapeMode.NONE
        )

        self.trigger_zone = (
            CollisionZone.NONE
        )

        self.clear_since = None

        self.set_state(
            RecoveryState.COOLDOWN,
            reason
        )

    # ================================================================
    # SWITCH DIRECTION
    # ================================================================

    def switch_direction(
        self,
        normal
    ):

        max_switches = int(
            self.get_parameter(
                'max_direction_switches'
            ).value
        )

        self.publisher.publish(
            self.stop_command()
        )

        if (
            self.direction_switch_count
            >= max_switches
        ):

            self.finish_recovery(
                'maximum direction switches reached'
            )

            return

        if self.escape_is_backward():

            if self.front_blocked(
                normal
            ):

                self.finish_recovery(
                    'front and rear blocked'
                )

                return

            self.escape_mode = (
                EscapeMode.FORWARD
            )

        elif self.escape_is_forward():

            if self.rear_blocked(
                normal
            ):

                self.finish_recovery(
                    'front and rear blocked'
                )

                return

            self.escape_mode = (
                EscapeMode.BACKWARD
            )

        else:

            self.finish_recovery(
                'cannot invert turn recovery'
            )

            return

        self.direction_switch_count += 1

        self.set_state(
            RecoveryState.PAUSE,
            'switching recovery direction'
        )

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ============================================================
        # SCAN WATCHDOG
        # ============================================================

        if not self.is_fresh(
            self.latest_scan_time,
            'scan_timeout'
        ):

            self.publisher.publish(
                self.stop_command()
            )

            return

        # ============================================================
        # CMD WATCHDOG
        # seulement en NORMAL
        # ============================================================

        if (
            self.state
            == RecoveryState.NORMAL

            and

            not self.is_fresh(
                self.latest_cmd_time,
                'command_timeout'
            )
        ):

            self.publisher.publish(
                self.stop_command()
            )

            return

        normal, recovery = (
            self.count_zone_points()
        )

        min_linear = float(
            self.get_parameter(
                'minimum_linear_command'
            ).value
        )

        min_angular = float(
            self.get_parameter(
                'minimum_angular_command'
            ).value
        )

        moving_forward = (
            self.latest_cmd.linear.x
            > min_linear
        )

        moving_backward = (
            self.latest_cmd.linear.x
            < -min_linear
        )

        turning_left = (
            abs(
                self.latest_cmd.linear.x
            ) <= min_linear

            and

            self.latest_cmd.angular.z
            > min_angular
        )

        turning_right = (
            abs(
                self.latest_cmd.linear.x
            ) <= min_linear

            and

            self.latest_cmd.angular.z
            < -min_angular
        )

        # ============================================================
        # NORMAL
        # ============================================================

        if (
            self.state
            == RecoveryState.NORMAL
        ):

            # --------------------------------------------------------
            # FORWARD
            # --------------------------------------------------------

            if moving_forward:

                zone = self.choose_zone(
                    normal,
                    [
                        CollisionZone.FRONT,
                        CollisionZone.LEFT_FRONT,
                        CollisionZone.RIGHT_FRONT
                    ]
                )

                if (
                    zone
                    != CollisionZone.NONE
                ):

                    mode = (
                        self.escape_for_zone(
                            zone
                        )
                    )

                    self.start_recovery(
                        mode,
                        zone,
                        f'{zone.name} collision '
                        f'({normal[zone]} points)'
                    )

                    return

            # --------------------------------------------------------
            # BACKWARD
            # --------------------------------------------------------

            if moving_backward:

                zone = self.choose_zone(
                    normal,
                    [
                        CollisionZone.REAR,
                        CollisionZone.LEFT_REAR,
                        CollisionZone.RIGHT_REAR
                    ]
                )

                if (
                    zone
                    != CollisionZone.NONE
                ):

                    mode = (
                        self.escape_for_zone(
                            zone
                        )
                    )

                    self.start_recovery(
                        mode,
                        zone,
                        f'{zone.name} collision '
                        f'({normal[zone]} points)'
                    )

                    return

            # --------------------------------------------------------
            # ROTATION LEFT
            # --------------------------------------------------------

            if turning_left:

                zone = self.choose_zone(
                    normal,
                    [
                        CollisionZone.LEFT_FRONT,
                        CollisionZone.RIGHT_REAR,
                        CollisionZone.FRONT,
                        CollisionZone.REAR
                    ]
                )

                if (
                    zone
                    != CollisionZone.NONE
                ):

                    self.start_recovery(
                        EscapeMode.TURN_RIGHT,
                        zone,
                        f'collision while turning left '
                        f'in {zone.name}'
                    )

                    return

            # --------------------------------------------------------
            # ROTATION RIGHT
            # --------------------------------------------------------

            if turning_right:

                zone = self.choose_zone(
                    normal,
                    [
                        CollisionZone.RIGHT_FRONT,
                        CollisionZone.LEFT_REAR,
                        CollisionZone.FRONT,
                        CollisionZone.REAR
                    ]
                )

                if (
                    zone
                    != CollisionZone.NONE
                ):

                    self.start_recovery(
                        EscapeMode.TURN_LEFT,
                        zone,
                        f'collision while turning right '
                        f'in {zone.name}'
                    )

                    return

            self.publisher.publish(
                self.latest_cmd
            )

            return

        # ============================================================
        # PAUSE
        # ============================================================

        if (
            self.state
            == RecoveryState.PAUSE
        ):

            self.publisher.publish(
                self.stop_command()
            )

            if (
                self.escape_is_backward()

                and

                self.rear_blocked(
                    normal
                )
            ):

                self.switch_direction(
                    normal
                )

                return

            if (
                self.escape_is_forward()

                and

                self.front_blocked(
                    normal
                )
            ):

                self.switch_direction(
                    normal
                )

                return

            pause = float(
                self.get_parameter(
                    'pause_duration'
                ).value
            )

            if (
                self.elapsed()
                >= pause
            ):

                self.set_state(
                    RecoveryState.ESCAPING,
                    f'escaping with '
                    f'{self.escape_mode.name}'
                )

            return

        # ============================================================
        # ESCAPING
        # ============================================================

        if (
            self.state
            == RecoveryState.ESCAPING
        ):

            if (
                self.escape_is_backward()

                and

                self.rear_blocked(
                    normal
                )
            ):

                self.switch_direction(
                    normal
                )

                return

            if (
                self.escape_is_forward()

                and

                self.front_blocked(
                    normal
                )
            ):

                self.switch_direction(
                    normal
                )

                return

            if self.recovery_zone_clear(
                recovery
            ):

                self.finish_recovery(
                    f'{self.trigger_zone.name} '
                    f'recovery zone clear'
                )

                return

            if (
                self.elapsed()
                >= self.max_escape_duration()
            ):

                self.finish_recovery(
                    'maximum recovery duration reached'
                )

                return

            self.publisher.publish(
                self.escape_command()
            )

            return

        # ============================================================
        # COOLDOWN
        # ============================================================

        if (
            self.state
            == RecoveryState.COOLDOWN
        ):

            self.publisher.publish(
                self.stop_command()
            )

            cooldown = float(
                self.get_parameter(
                    'cooldown_duration'
                ).value
            )

            if (
                self.elapsed()
                >= cooldown
            ):

                self.direction_switch_count = 0
                self.clear_since = None

                self.set_state(
                    RecoveryState.NORMAL,
                    'control returned to Nav2'
                )

            return


def main(args=None):

    rclpy.init(
        args=args
    )

    node = CollisionRecoveryNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.publisher.publish(
            Twist()
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
