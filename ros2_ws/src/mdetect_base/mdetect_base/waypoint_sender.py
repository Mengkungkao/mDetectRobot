#!/usr/bin/env python3
"""One-shot command-line publisher for timed waypoint lists."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('waypoint_sender')
    publisher = node.create_publisher(String, '/waypoint_input', 10)

    ros_args_removed = remove_ros_args(args=sys.argv)
    if len(ros_args_removed) < 2:
        node.get_logger().error(
            'Usage: ros2 run mdetect_base send_waypoints '
            '"(0,2000,0,0) (1000,2000,0,5)"')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    text = ' '.join(ros_args_removed[1:])
    msg = String()
    msg.data = text

    # Wait for DDS discovery, then publish exactly once. Repeated copies would
    # otherwise replace or cancel a waypoint list that is already executing.
    discovery_deadline = time.monotonic() + 3.0
    while (
        rclpy.ok() and
        publisher.get_subscription_count() == 0 and
        time.monotonic() < discovery_deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.05)

    if publisher.get_subscription_count() == 0:
        node.get_logger().error(
            'No /waypoint_input subscriber was discovered. '
            'Start timed_waypoint_executor and check ROS_DOMAIN_ID/network settings.')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    publisher.publish(msg)
    # Give the middleware a brief opportunity to transmit before shutting down.
    end_time = time.monotonic() + 0.5
    while rclpy.ok() and time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.get_logger().info(f'Published waypoint list: {text}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
