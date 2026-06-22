#!/usr/bin/env python3
"""Low-speed straight-line report for the mDetect base."""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray


def quaternion_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def angle_delta_deg(end: float, start: float) -> float:
    return (end - start + 180.0) % 360.0 - 180.0


class StraightLineTest(Node):
    def __init__(self, speed: float, duration: float) -> None:
        super().__init__('mdetect_straight_line_test')
        self.speed = speed
        self.duration = duration
        self.started = time.monotonic()
        self.finished = False
        self.blocked = False
        self.odom_samples: list[tuple[float, float, float]] = []
        self.imu_yaw: list[float] = []
        self.wheel_samples: list[list[float]] = []
        self.publisher = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 20)
        self.create_subscription(Float32MultiArray, '/base/wheel_speeds_mm_s', self.on_wheels, 20)
        self.create_subscription(Bool, '/safety/front_blocked', self.on_blocked, 10)
        self.create_timer(0.1, self.tick)

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.odom_samples.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_yaw_deg(q.x, q.y, q.z, q.w),
        ))

    def on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self.imu_yaw.append(quaternion_yaw_deg(q.x, q.y, q.z, q.w))

    def on_wheels(self, msg: Float32MultiArray) -> None:
        if len(msg.data) == 4 and time.monotonic() - self.started > 0.8:
            self.wheel_samples.append([float(value) for value in msg.data])

    def on_blocked(self, msg: Bool) -> None:
        self.blocked = msg.data

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def tick(self) -> None:
        elapsed = time.monotonic() - self.started
        if elapsed < self.duration and not self.blocked:
            command = Twist()
            command.linear.x = self.speed
            self.publisher.publish(command)
            return
        self.publish_stop()
        if elapsed >= self.duration + 0.6 or self.blocked:
            self.finished = True

    def report(self) -> int:
        print('\n=== mDetect straight-line report ===')
        if self.blocked:
            print('ABORTED: front safety gate detected an obstacle.')
            return 2
        if len(self.odom_samples) < 2:
            print('FAIL: no usable /odom samples were received.')
            return 1

        x0, y0, yaw0 = self.odom_samples[0]
        x1, y1, yaw1 = self.odom_samples[-1]
        distance = math.hypot(x1 - x0, y1 - y0)
        yaw_drift = angle_delta_deg(yaw1, yaw0)
        print(f'Distance travelled: {distance:.3f} m')
        print(f'Lateral odometry change: {y1 - y0:+.3f} m')
        print(f'Yaw drift: {yaw_drift:+.2f} deg')

        if self.wheel_samples:
            averages = [
                statistics.fmean(abs(sample[index]) for sample in self.wheel_samples)
                for index in range(4)
            ]
            median_speed = statistics.median(averages)
            names = ['M1 front-left', 'M2 front-right', 'M3 rear-right', 'M4 rear-left']
            print('\nAverage measured wheel speeds:')
            for name, measured in zip(names, averages):
                multiplier = median_speed / measured if measured > 1.0 else 1.0
                print(f'  {name}: {measured:7.1f} mm/s  relative trim multiplier {multiplier:.3f}')
            spread = max(averages) - min(averages)
            print(f'Wheel-speed spread: {spread:.1f} mm/s')
        else:
            print('No wheel-speed samples were captured.')

        if abs(yaw_drift) <= 3.0:
            print('\nPASS: straight-line yaw drift is within 3 degrees for this short test.')
            return 0
        print('\nCHECK: yaw drift is above 3 degrees. Review IMU sign/calibration and motor trims.')
        return 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--confirm-clear-path', action='store_true')
    parser.add_argument('--speed', type=float, default=0.08)
    parser.add_argument('--duration', type=float, default=3.0)
    args = parser.parse_args()
    if not args.confirm_clear_path:
        parser.error('Use --confirm-clear-path after providing at least 2 m of clear floor.')
    if not 0.03 <= abs(args.speed) <= 0.15:
        parser.error('Test speed must be between 0.03 and 0.15 m/s.')
    if not 1.0 <= args.duration <= 8.0:
        parser.error('Duration must be between 1 and 8 seconds.')

    rclpy.init()
    node = StraightLineTest(args.speed, args.duration)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.05)
        return node.report()
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
