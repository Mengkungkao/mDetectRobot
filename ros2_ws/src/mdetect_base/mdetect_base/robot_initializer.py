#!/usr/bin/env python3
"""Initialize the low-level controller and report when essential robot data is alive."""
from __future__ import annotations

import time
from typing import Dict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class RobotInitializer(Node):
    def __init__(self) -> None:
        super().__init__('mdetect_robot_initializer')
        self.declare_parameter('initialize_on_start', True)
        self.declare_parameter('startup_delay_s', 2.0)
        self.declare_parameter('ready_timeout_s', 20.0)

        self.start_time = time.monotonic()
        self.initialize_on_start = bool(self.get_parameter('initialize_on_start').value)
        self.startup_delay = float(self.get_parameter('startup_delay_s').value)
        self.ready_timeout = float(self.get_parameter('ready_timeout_s').value)
        self.initialization_sent = False
        self.ready_announced = False
        self.seen: Dict[str, float] = {'scan': 0.0, 'odom': 0.0, 'imu': 0.0, 'joint_states': 0.0}

        self.ready_pub = self.create_publisher(Bool, '/robot/ready', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(LaserScan, '/scan', lambda _m: self.mark('scan'), 10)
        self.create_subscription(Odometry, '/odom', lambda _m: self.mark('odom'), 10)
        self.create_subscription(Imu, '/imu/data', lambda _m: self.mark('imu'), 10)
        self.create_subscription(JointState, '/joint_states', lambda _m: self.mark('joint_states'), 10)

        self.reset_client = self.create_client(Trigger, '/base/reset_odometry')
        self.zero_client = self.create_client(Trigger, '/base/zero_yaw')
        self.create_timer(0.5, self.tick)

    def mark(self, name: str) -> None:
        self.seen[name] = time.monotonic()

    def call_trigger(self, client, name: str) -> None:
        if client.service_is_ready():
            client.call_async(Trigger.Request())
            self.get_logger().info(f'Initialization command sent: {name}')
        else:
            self.get_logger().warning(f'Initialization service not ready: {name}')

    def tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self.start_time

        if self.initialize_on_start and not self.initialization_sent and elapsed >= self.startup_delay:
            self.call_trigger(self.reset_client, 'reset odometry')
            self.call_trigger(self.zero_client, 'zero IMU yaw')
            self.initialization_sent = True

        alive = {name: stamp > 0.0 and now - stamp < 2.0 for name, stamp in self.seen.items()}
        ready = all(alive.values())
        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_pub.publish(ready_msg)

        status = DiagnosticStatus()
        status.name = 'mdetect/startup'
        status.hardware_id = 'mdetect-robot'
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.WARN
        status.message = 'Robot ready' if ready else 'Waiting for: ' + ', '.join(k for k, v in alive.items() if not v)
        status.values = [KeyValue(key=name, value='alive' if value else 'missing') for name, value in alive.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)

        if ready and not self.ready_announced:
            self.get_logger().info('ROBOT READY: Pi, Arduino, encoders, IMU, LiDAR, TF and motor command path are initialized.')
            self.ready_announced = True
        elif not ready and elapsed > self.ready_timeout and not self.ready_announced:
            self.get_logger().error(status.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
