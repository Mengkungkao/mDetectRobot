#!/usr/bin/env python3
"""Run named waypoint routes, including a wait time at each waypoint."""
from __future__ import annotations

import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class WaypointCLI(Node):
    def __init__(self) -> None:
        super().__init__('mdetect_waypoint_cli')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def make_pose(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        yaw = math.radians(float(yaw_deg))
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def run(self) -> None:
        path = os.path.join(get_package_share_directory('mdetect_navigation'), 'config', 'waypoints.yaml')
        with open(path, encoding='utf-8') as stream:
            routes = yaml.safe_load(stream)['routes']

        names = list(routes)
        print('\nPredefined routes:')
        for index, name in enumerate(names, start=1):
            print(f'  {index}. {name}')
        selected = names[int(input('Select route number: ').strip()) - 1]

        print('Waiting for Nav2 NavigateToPose action...')
        self.client.wait_for_server()
        for number, waypoint in enumerate(routes[selected], start=1):
            if len(waypoint) == 3:
                x, y, yaw = waypoint
                wait_s = 0.0
            else:
                x, y, yaw, wait_s = waypoint
            goal = NavigateToPose.Goal()
            goal.pose = self.make_pose(x, y, yaw)
            print(f'Waypoint {number}: x={x}, y={y}, yaw={yaw}°, wait={wait_s}s')
            sent = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, sent)
            handle = sent.result()
            if handle is None or not handle.accepted:
                print(f'Waypoint {number} was rejected; route stopped.')
                return
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result)
            if result.result() is None:
                print(f'Waypoint {number} failed; route stopped.')
                return
            if float(wait_s) > 0.0:
                time.sleep(float(wait_s))
        print(f'Route {selected} finished.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointCLI()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
