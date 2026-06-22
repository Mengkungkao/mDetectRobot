#!/usr/bin/env python3
"""TurtleBot-style velocity priority mux with a front LiDAR safety gate."""

from __future__ import annotations

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class CmdMux(Node):
    def __init__(self) -> None:
        super().__init__('mdetect_cmd_mux')
        self.declare_parameter('front_stop_distance', 0.30)
        self.declare_parameter('front_half_angle_deg', 45.0)
        self.declare_parameter('source_timeout', 0.40)
        self.declare_parameter('scan_timeout', 0.75)
        self.declare_parameter('allow_rotation_when_blocked', True)

        self.stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_half_angle = math.radians(
            float(self.get_parameter('front_half_angle_deg').value)
        )
        self.source_timeout = float(self.get_parameter('source_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.allow_rotation_when_blocked = bool(
            self.get_parameter('allow_rotation_when_blocked').value
        )

        self.sources: dict[str, tuple[Twist, float]] = {
            'manual': (Twist(), 0.0),
            'teleop': (Twist(), 0.0),
            'nav': (Twist(), 0.0),
        }
        self.blocked = False
        self.nearest_front = math.inf
        self.last_scan_time = 0.0
        self.active_source = 'none'

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        self.blocked_pub = self.create_publisher(Bool, '/safety/front_blocked', 10)
        self.distance_pub = self.create_publisher(Float32, '/safety/front_distance', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(
            Twist, '/cmd_vel_manual', lambda msg: self.set_source('manual', msg), 20
        )
        self.create_subscription(
            Twist, '/cmd_vel_teleop', lambda msg: self.set_source('teleop', msg), 20
        )
        self.create_subscription(
            Twist, '/cmd_vel_nav', lambda msg: self.set_source('nav', msg), 20
        )
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.create_timer(0.05, self.publish_command)
        self.create_timer(1.0, self.publish_diagnostics)

    def set_source(self, name: str, msg: Twist) -> None:
        self.sources[name] = (msg, time.monotonic())

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_callback(self, msg: LaserScan) -> None:
        nearest = math.inf
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            angle = self.normalize_angle(msg.angle_min + index * msg.angle_increment)
            if abs(angle) <= self.front_half_angle:
                nearest = min(nearest, distance)

        self.last_scan_time = time.monotonic()
        self.nearest_front = nearest
        self.blocked = nearest < self.stop_distance

        blocked_msg = Bool()
        blocked_msg.data = self.blocked
        self.blocked_pub.publish(blocked_msg)

        distance_msg = Float32()
        distance_msg.data = float(nearest) if math.isfinite(nearest) else float('inf')
        self.distance_pub.publish(distance_msg)

    def choose_command(self) -> Twist:
        now = time.monotonic()
        # Manual service/test commands have highest priority, then keyboard teleop,
        # then Nav2. This mirrors TurtleBot-style operator override behaviour.
        for source_name in ('manual', 'teleop', 'nav'):
            msg, timestamp = self.sources[source_name]
            if now - timestamp <= self.source_timeout:
                self.active_source = source_name
                return msg
        self.active_source = 'none'
        return Twist()

    def publish_command(self) -> None:
        output = self.choose_command()
        safe = Twist()
        safe.linear.x = output.linear.x
        safe.linear.y = output.linear.y
        safe.linear.z = output.linear.z
        safe.angular.x = output.angular.x
        safe.angular.y = output.angular.y
        safe.angular.z = output.angular.z

        scan_stale = (
            self.last_scan_time == 0.0
            or time.monotonic() - self.last_scan_time > self.scan_timeout
        )

        # Stop forward motion for a close obstacle. Rotation remains available so
        # Nav2 or an operator can turn away from the obstacle.
        if self.blocked and safe.linear.x > 0.0:
            safe.linear.x = 0.0
            if not self.allow_rotation_when_blocked:
                safe.angular.z = 0.0

        # A missing LiDAR does not silently disable the drive base, because the
        # Arduino watchdog still protects communications. It is clearly reported
        # through diagnostics and the verification scripts.
        if scan_stale:
            self.get_logger().debug('LiDAR scan is stale')

        self.cmd_pub.publish(safe)

    def publish_diagnostics(self) -> None:
        now = time.monotonic()
        scan_age = now - self.last_scan_time if self.last_scan_time else math.inf
        status = DiagnosticStatus()
        status.name = 'mDetect velocity mux and front safety gate'
        status.hardware_id = 'mdetect_cmd_mux'

        if scan_age > self.scan_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = 'LiDAR scan missing or stale'
        elif self.blocked:
            status.level = DiagnosticStatus.WARN
            status.message = 'Forward motion blocked by obstacle'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Velocity mux operational'

        status.values = [
            KeyValue(key='active_source', value=self.active_source),
            KeyValue(key='front_blocked', value=str(self.blocked)),
            KeyValue(
                key='front_distance_m',
                value='inf' if not math.isfinite(self.nearest_front) else f'{self.nearest_front:.3f}',
            ),
            KeyValue(
                key='scan_age_s',
                value='inf' if math.isinf(scan_age) else f'{scan_age:.3f}',
            ),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
