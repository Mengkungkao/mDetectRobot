#!/usr/bin/env python3
"""Independent front-obstacle and command-timeout safety layer."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger


class SafetySupervisor(Node):
    def __init__(self) -> None:
        super().__init__('safety_supervisor')

        self.declare_parameter('command_input_topic', '/cmd_vel')
        self.declare_parameter('command_output_topic', '/cmd_vel_safe')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_half_angle_deg', 40.0)
        self.declare_parameter('stop_distance', 0.25)
        self.declare_parameter('slowdown_distance', 0.50)
        self.declare_parameter('clear_distance', 0.35)
        self.declare_parameter('command_timeout', 0.30)
        self.declare_parameter('scan_timeout', 0.50)
        self.declare_parameter('stop_on_scan_timeout', True)
        self.declare_parameter('latch_on_obstacle', True)
        self.declare_parameter('min_turn_scale', 0.30)

        self.command_input_topic = str(self.get_parameter('command_input_topic').value)
        self.command_output_topic = str(self.get_parameter('command_output_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.front_half_angle = math.radians(
            float(self.get_parameter('front_half_angle_deg').value)
        )
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.stop_on_scan_timeout = bool(self.get_parameter('stop_on_scan_timeout').value)
        self.latch_on_obstacle = bool(self.get_parameter('latch_on_obstacle').value)
        self.min_turn_scale = float(self.get_parameter('min_turn_scale').value)

        if self.stop_distance <= 0.0 or self.slowdown_distance <= self.stop_distance:
            raise ValueError('slowdown_distance must be greater than stop_distance > 0')
        if self.clear_distance < self.stop_distance:
            raise ValueError('clear_distance must be at least stop_distance')

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.cmd_sub = self.create_subscription(
            Twist, self.command_input_topic, self.command_callback, 20)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.safe_pub = self.create_publisher(Twist, self.command_output_topic, 20)
        self.estop_pub = self.create_publisher(Bool, '/safety/emergency_stop', 10)
        self.range_pub = self.create_publisher(Float32, '/safety/min_front_range', 10)
        self.status_pub = self.create_publisher(String, '/safety/status', 10)

        self.estop_client = self.create_client(Trigger, '/base/emergency_stop')
        self.clear_client = self.create_client(Trigger, '/base/clear_emergency_stop')
        self.brake_client = self.create_client(Trigger, '/base/brake')
        self.create_service(Trigger, '/safety/clear', self.clear_callback)
        self.create_service(Trigger, '/safety/stop', self.manual_stop_callback)

        self.latest_cmd = Twist()
        self.last_cmd_ns: Optional[int] = None
        self.last_scan_ns: Optional[int] = None
        self.min_front_range = math.inf
        self.latched = False
        self.estop_request_sent = False
        self.timeout_brake_sent = False
        self.reason = 'startup'

        self.timer = self.create_timer(0.02, self.control_cycle)
        self.get_logger().info(
            f'Safety supervisor: stop={self.stop_distance:.2f} m, '
            f'slow={self.slowdown_distance:.2f} m, '
            f'cone=±{math.degrees(self.front_half_angle):.1f}°'
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def command_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_ns = self.now_ns()
        self.timeout_brake_sent = False

    def scan_callback(self, msg: LaserScan) -> None:
        minimum = math.inf
        angle = msg.angle_min
        for distance in msg.ranges:
            if abs(angle) <= self.front_half_angle:
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    minimum = min(minimum, float(distance))
            angle += msg.angle_increment

        self.min_front_range = minimum
        self.last_scan_ns = self.now_ns()
        range_msg = Float32()
        range_msg.data = minimum if math.isfinite(minimum) else -1.0
        self.range_pub.publish(range_msg)

    def copy_twist(self, source: Twist) -> Twist:
        output = Twist()
        output.linear.x = source.linear.x
        output.linear.y = source.linear.y
        output.linear.z = source.linear.z
        output.angular.x = source.angular.x
        output.angular.y = source.angular.y
        output.angular.z = source.angular.z
        return output

    def moving(self, command: Twist) -> bool:
        return (
            abs(command.linear.x) > 1e-4 or
            abs(command.linear.y) > 1e-4 or
            abs(command.angular.z) > 1e-4
        )

    def moving_forward(self, command: Twist) -> bool:
        return command.linear.x > 1e-4

    def call_trigger(self, client) -> None:
        if client.service_is_ready():
            client.call_async(Trigger.Request())

    def latch_stop(self, reason: str) -> None:
        if not self.latched:
            self.get_logger().error(f'Safety stop latched: {reason}')
        self.latched = True
        self.reason = reason
        if not self.estop_request_sent:
            self.call_trigger(self.estop_client)
            self.estop_request_sent = True

    def control_cycle(self) -> None:
        now = self.now_ns()
        cmd_stale = (
            self.last_cmd_ns is None or
            now - self.last_cmd_ns > int(self.command_timeout * 1e9)
        )
        scan_stale = (
            self.last_scan_ns is None or
            now - self.last_scan_ns > int(self.scan_timeout * 1e9)
        )

        command = Twist() if cmd_stale else self.copy_twist(self.latest_cmd)

        if cmd_stale and not self.timeout_brake_sent:
            self.call_trigger(self.brake_client)
            self.timeout_brake_sent = True
            self.reason = 'command timeout'

        if self.moving(command):
            if scan_stale and self.stop_on_scan_timeout:
                self.latch_stop('LiDAR scan timeout while movement was requested')
            elif self.moving_forward(command) and self.min_front_range <= self.stop_distance:
                if self.latch_on_obstacle:
                    self.latch_stop(
                        f'front obstacle at {self.min_front_range:.3f} m'
                    )
                else:
                    command = Twist()
                    self.call_trigger(self.brake_client)
                    self.reason = 'front obstacle stop'

        if self.latched:
            command = Twist()
        elif (
            self.moving_forward(command) and
            math.isfinite(self.min_front_range) and
            self.stop_distance < self.min_front_range < self.slowdown_distance
        ):
            scale = (
                (self.min_front_range - self.stop_distance) /
                (self.slowdown_distance - self.stop_distance)
            )
            scale = max(0.0, min(1.0, scale))
            command.linear.x *= scale
            command.linear.y *= scale
            command.angular.z *= max(scale, self.min_turn_scale)
            self.reason = f'slowing for obstacle at {self.min_front_range:.3f} m'
        elif not cmd_stale:
            self.reason = 'clear'

        self.safe_pub.publish(command)

        estop = Bool()
        estop.data = self.latched
        self.estop_pub.publish(estop)

        status = String()
        status.data = self.reason
        self.status_pub.publish(status)

    def clear_callback(self, request, response):
        del request
        scan_fresh = (
            self.last_scan_ns is not None and
            self.now_ns() - self.last_scan_ns <= int(self.scan_timeout * 1e9)
        )
        clear = (
            scan_fresh and
            (not math.isfinite(self.min_front_range) or self.min_front_range >= self.clear_distance)
        )

        if not clear:
            response.success = False
            response.message = (
                f'Cannot clear: front range is {self.min_front_range:.3f} m or scan is stale'
            )
            return response

        self.latched = False
        self.estop_request_sent = False
        self.reason = 'cleared by user'
        self.call_trigger(self.clear_client)
        response.success = True
        response.message = 'Safety latch cleared; send a new navigation command'
        return response

    def manual_stop_callback(self, request, response):
        del request
        self.latch_stop('manual ROS safety stop')
        response.success = True
        response.message = 'Safety stop latched'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
