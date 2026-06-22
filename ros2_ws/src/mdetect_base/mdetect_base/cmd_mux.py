#!/usr/bin/env python3
"""Velocity priority mux with an independent front LiDAR emergency stop gate."""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class CmdMux(Node):
    def __init__(self) -> None:
        super().__init__('mdetect_cmd_mux')
        self.declare_parameter('front_stop_distance', 0.30)
        self.declare_parameter('front_release_distance', 0.36)
        self.declare_parameter('front_half_angle_deg', 45.0)
        self.declare_parameter('source_timeout', 0.40)
        self.declare_parameter('scan_timeout', 0.75)
        self.declare_parameter('stop_on_scan_timeout', True)

        self.stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.release_distance = float(self.get_parameter('front_release_distance').value)
        self.half_angle = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        self.source_timeout = float(self.get_parameter('source_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.stop_on_scan_timeout = bool(self.get_parameter('stop_on_scan_timeout').value)

        self.sources = {
            'manual': (Twist(), 0.0),
            'teleop': (Twist(), 0.0),
            'nav': (Twist(), 0.0),
        }
        self.front_blocked = False
        self.last_scan_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        self.blocked_pub = self.create_publisher(Bool, '/safety/front_blocked', 10)
        self.create_subscription(Twist, '/cmd_vel_manual', lambda msg: self.set_source('manual', msg), 20)
        self.create_subscription(Twist, '/cmd_vel_teleop', lambda msg: self.set_source('teleop', msg), 20)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda msg: self.set_source('nav', msg), 20)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_timer(0.05, self.publish_command)

    def set_source(self, name: str, msg: Twist) -> None:
        self.sources[name] = (msg, time.monotonic())

    def scan_callback(self, msg: LaserScan) -> None:
        nearest = float('inf')
        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) <= self.half_angle and math.isfinite(distance) and distance >= msg.range_min:
                nearest = min(nearest, distance)

        self.last_scan_time = time.monotonic()
        if self.front_blocked:
            self.front_blocked = nearest < self.release_distance
        else:
            self.front_blocked = nearest < self.stop_distance

    def publish_command(self) -> None:
        now = time.monotonic()
        output = Twist()
        for source in ('manual', 'teleop', 'nav'):
            msg, stamp = self.sources[source]
            if now - stamp <= self.source_timeout:
                output = msg
                break

        scan_stale = self.last_scan_time == 0.0 or now - self.last_scan_time > self.scan_timeout
        blocked = self.front_blocked or (self.stop_on_scan_timeout and scan_stale)
        if blocked and output.linear.x > 0.0:
            output = Twist()

        status = Bool()
        status.data = blocked
        self.blocked_pub.publish(status)
        self.cmd_pub.publish(output)


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
