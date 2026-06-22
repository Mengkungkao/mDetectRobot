#!/usr/bin/env python3
"""Bench test all four motors and encoders through the normal ROS command path."""
from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class ActuatorSelfTest(Node):
    def __init__(self) -> None:
        super().__init__('mdetect_actuator_self_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.create_subscription(String, '/base/arduino_rx', self.telemetry_callback, 50)
        self.start = time.monotonic()
        self.forward_max = [0.0] * 4
        self.reverse_max = [0.0] * 4
        self.latest = [0.0] * 4
        self.finished = False
        self.passed = False
        self.create_timer(0.05, self.tick)

    def telemetry_callback(self, msg: String) -> None:
        fields = msg.data.split(',')
        if len(fields) != 21 or fields[0] != 'T':
            return
        try:
            self.latest = [float(value) for value in fields[11:15]]
        except ValueError:
            return

    def publish_velocity(self, linear: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        self.publisher.publish(msg)

    def tick(self) -> None:
        elapsed = time.monotonic() - self.start
        if elapsed < 1.0:
            self.publish_velocity(0.0)
        elif elapsed < 3.0:
            self.publish_velocity(0.08)
            self.forward_max = [max(old, value) for old, value in zip(self.forward_max, self.latest)]
        elif elapsed < 4.3:
            self.publish_velocity(0.0)
        elif elapsed < 6.3:
            self.publish_velocity(-0.08)
            self.reverse_max = [min(old, value) for old, value in zip(self.reverse_max, self.latest)]
        elif elapsed < 7.5:
            self.publish_velocity(0.0)
        else:
            self.finish()

    def finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        self.publish_velocity(0.0)
        minimum = 20.0
        forward_ok = [speed > minimum for speed in self.forward_max]
        reverse_ok = [speed < -minimum for speed in self.reverse_max]
        self.passed = all(forward_ok) and all(reverse_ok)
        print('\nActuator self-test results (mm/s):')
        for index in range(4):
            state = 'PASS' if forward_ok[index] and reverse_ok[index] else 'FAIL'
            print(
                f'  Motor {index + 1}: forward={self.forward_max[index]:7.1f}, '
                f'reverse={self.reverse_max[index]:7.1f}  {state}'
            )
        if self.passed:
            print('PASS: all four motors and encoder directions responded.')
        else:
            print('FAIL: one or more motors/encoders did not reach the expected signed speed.')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActuatorSelfTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_velocity(0.0)
        node.passed = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if node.passed else 1)


if __name__ == '__main__':
    main()
