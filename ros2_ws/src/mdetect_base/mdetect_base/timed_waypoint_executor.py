#!/usr/bin/env python3
"""Execute textual (x, y, heading, wait) waypoint lists through Nav2."""

import math
import re
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger


@dataclass
class TimedWaypoint:
    source_x: float
    source_y: float
    source_heading_deg: float
    wait_seconds: float
    ros_x: float
    ros_y: float
    ros_yaw: float


def quaternion_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class TimedWaypointExecutor(Node):
    def __init__(self) -> None:
        super().__init__('timed_waypoint_executor')

        self.declare_parameter('input_topic', '/waypoint_input')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter(
            'input_coordinate_mode', 'legacy_x_right_y_forward')
        self.declare_parameter('input_distance_unit', 'mm')
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('server_wait_timeout', 30.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.coordinate_mode = str(
            self.get_parameter('input_coordinate_mode').value).lower()
        self.distance_unit = str(
            self.get_parameter('input_distance_unit').value).lower()
        self.stop_on_failure = bool(self.get_parameter('stop_on_failure').value)
        self.server_wait_timeout = float(
            self.get_parameter('server_wait_timeout').value)

        if self.distance_unit == 'mm':
            self.distance_scale = 0.001
        elif self.distance_unit == 'm':
            self.distance_scale = 1.0
        else:
            raise ValueError("input_distance_unit must be 'mm' or 'm'")

        supported_modes = {
            'legacy_x_right_y_forward',
            'ros_x_forward_y_left',
        }
        if self.coordinate_mode not in supported_modes:
            raise ValueError(
                f'input_coordinate_mode must be one of {sorted(supported_modes)}')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.input_sub = self.create_subscription(
            String, self.input_topic, self.input_callback, 10)
        self.safety_sub = self.create_subscription(
            Bool, '/safety/emergency_stop', self.safety_callback, 10)
        self.status_pub = self.create_publisher(String, '/waypoints/status', 10)
        self.index_pub = self.create_publisher(Int32, '/waypoints/current_index', 10)
        self.path_pub = self.create_publisher(Path, '/waypoints/requested_path', 10)
        self.create_service(Trigger, '/waypoints/cancel', self.cancel_callback)

        self.waypoints: List[TimedWaypoint] = []
        self.pending_replacement: Optional[List[TimedWaypoint]] = None
        self.current_index = 0
        self.goal_handle = None
        self.goal_result_future = None
        self.state = 'IDLE'
        self.wait_until_ns: Optional[int] = None
        self.sequence_start_ns: Optional[int] = None
        self.server_deadline_ns: Optional[int] = None
        self.cancel_is_user_request = False
        self.safety_latched = False
        self.generation = 0
        self.timer = self.create_timer(0.05, self.tick)

        self.publish_status(
            'Ready. Publish: (x,y,heading_deg,wait_s) (x,y,heading_deg,wait_s)')

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def parse_input(self, text: str) -> List[TimedWaypoint]:
        groups = re.findall(r'\(([^()]*)\)', text)
        if not groups:
            raise ValueError('No waypoint groups found')

        result: List[TimedWaypoint] = []
        for number, group in enumerate(groups, start=1):
            parts = [part.strip() for part in group.split(',')]
            if len(parts) != 4:
                raise ValueError(
                    f'Waypoint {number} must contain x,y,heading,wait')
            try:
                x, y, heading, wait_seconds = [float(value) for value in parts]
            except ValueError as exc:
                raise ValueError(f'Waypoint {number} contains a non-number') from exc
            if wait_seconds < 0.0:
                raise ValueError(f'Waypoint {number} wait time cannot be negative')

            if self.coordinate_mode == 'legacy_x_right_y_forward':
                ros_x = y * self.distance_scale
                ros_y = -x * self.distance_scale
                ros_yaw = -math.radians(heading)
            else:
                ros_x = x * self.distance_scale
                ros_y = y * self.distance_scale
                ros_yaw = math.radians(heading)

            result.append(TimedWaypoint(
                source_x=x,
                source_y=y,
                source_heading_deg=heading,
                wait_seconds=wait_seconds,
                ros_x=ros_x,
                ros_y=ros_y,
                ros_yaw=ros_yaw,
            ))
        return result

    def input_callback(self, msg: String) -> None:
        try:
            parsed = self.parse_input(msg.data)
        except ValueError as exc:
            self.publish_status(f'Waypoint input rejected: {exc}')
            return

        if self.safety_latched:
            self.publish_status(
                'Waypoint input rejected because the safety stop is latched')
            return

        if self.goal_handle is not None:
            self.pending_replacement = parsed
            self.cancel_is_user_request = False
            self.state = 'CANCELLING_FOR_REPLACEMENT'
            self.goal_handle.cancel_goal_async()
            self.publish_status('Cancelling the active goal and loading the new list')
            return

        self.start_sequence(parsed)

    def start_sequence(self, waypoints: List[TimedWaypoint]) -> None:
        # Invalidate any goal-response callback that belongs to an older list.
        self.generation += 1
        self.waypoints = waypoints
        self.pending_replacement = None
        self.current_index = 0
        self.sequence_start_ns = self.now_ns()
        self.server_deadline_ns = self.now_ns() + int(self.server_wait_timeout * 1e9)
        self.state = 'WAITING_FOR_SERVER'
        self.wait_until_ns = None
        self.publish_requested_path()
        self.publish_status(f'Loaded {len(waypoints)} timed waypoints')

    def publish_requested_path(self) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for waypoint in self.waypoints:
            pose = self.pose_for_waypoint(waypoint)
            path.poses.append(pose)
        self.path_pub.publish(path)

    def pose_for_waypoint(self, waypoint: TimedWaypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = waypoint.ros_x
        pose.pose.position.y = waypoint.ros_y
        qx, qy, qz, qw = quaternion_from_yaw(waypoint.ros_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def tick(self) -> None:
        if self.state == 'WAITING_FOR_SERVER':
            if self.action_client.server_is_ready():
                self.send_current_goal()
            elif self.server_deadline_ns is not None and self.now_ns() >= self.server_deadline_ns:
                self.fail_sequence('Nav2 navigate_to_pose action server was not available')

        elif self.state == 'WAITING_AT_WAYPOINT':
            if self.wait_until_ns is not None and self.now_ns() >= self.wait_until_ns:
                self.current_index += 1
                if self.current_index >= len(self.waypoints):
                    elapsed = 0.0
                    if self.sequence_start_ns is not None:
                        elapsed = (self.now_ns() - self.sequence_start_ns) / 1e9
                    self.state = 'IDLE'
                    self.publish_status(
                        f'All {len(self.waypoints)} waypoints completed in {elapsed:.1f} s')
                else:
                    self.send_current_goal()

    def send_current_goal(self) -> None:
        if self.current_index >= len(self.waypoints):
            self.state = 'IDLE'
            return
        if self.safety_latched:
            self.fail_sequence('Safety stop became active before goal dispatch')
            return

        waypoint = self.waypoints[self.current_index]
        index_msg = Int32()
        index_msg.data = self.current_index
        self.index_pub.publish(index_msg)

        goal = NavigateToPose.Goal()
        goal.pose = self.pose_for_waypoint(waypoint)
        goal.behavior_tree = ''

        self.state = 'SENDING_GOAL'
        generation = self.generation
        goal_index = self.current_index
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(
            lambda completed, g=generation, i=goal_index:
            self.goal_response_callback(completed, g, i))
        self.publish_status(
            f'Sending waypoint {self.current_index + 1}/{len(self.waypoints)}: '
            f'ROS ({waypoint.ros_x:.3f}, {waypoint.ros_y:.3f}, '
            f'{math.degrees(waypoint.ros_yaw):.1f}°), wait {waypoint.wait_seconds:.1f} s')

    def goal_response_callback(self, future, generation: int, goal_index: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if generation == self.generation:
                self.fail_sequence(f'Failed to send Nav2 goal: {exc}')
            return

        if generation != self.generation or goal_index != self.current_index:
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.fail_sequence('Nav2 rejected the waypoint goal')
            return

        if self.safety_latched or not self.waypoints:
            goal_handle.cancel_goal_async()
            self.fail_sequence('Goal cancelled before movement because safety is active')
            return

        self.goal_handle = goal_handle
        self.state = 'NAVIGATING'
        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(
            lambda completed, g=generation, i=goal_index, h=goal_handle:
            self.goal_result_callback(completed, g, i, h))

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'distance_remaining'):
            # Keep console output useful without logging every feedback packet.
            pass

    def goal_result_callback(self, future, generation: int, goal_index: int, goal_handle) -> None:
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as exc:
            if self.goal_handle is goal_handle:
                self.goal_handle = None
                self.goal_result_future = None
            if generation == self.generation and goal_index == self.current_index:
                self.fail_sequence(f'Nav2 result failed: {exc}')
            return

        if self.goal_handle is goal_handle:
            self.goal_handle = None
            self.goal_result_future = None

        if generation != self.generation or goal_index != self.current_index:
            return

        if self.pending_replacement is not None:
            replacement = self.pending_replacement
            self.pending_replacement = None
            self.start_sequence(replacement)
            return

        if self.cancel_is_user_request or self.safety_latched:
            self.cancel_is_user_request = False
            self.waypoints = []
            self.state = 'IDLE'
            self.publish_status('Waypoint sequence cancelled')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            waypoint = self.waypoints[self.current_index]
            self.wait_until_ns = self.now_ns() + int(waypoint.wait_seconds * 1e9)
            self.state = 'WAITING_AT_WAYPOINT'
            self.publish_status(
                f'Waypoint {self.current_index + 1} reached; '
                f'waiting {waypoint.wait_seconds:.1f} s')
            return

        status_names = {
            GoalStatus.STATUS_ABORTED: 'aborted',
            GoalStatus.STATUS_CANCELED: 'cancelled',
            GoalStatus.STATUS_UNKNOWN: 'unknown',
        }
        reason = status_names.get(status, f'status {status}')
        if self.stop_on_failure:
            self.fail_sequence(
                f'Waypoint {self.current_index + 1} failed: {reason}')
        else:
            self.publish_status(
                f'Waypoint {self.current_index + 1} failed ({reason}); continuing')
            self.current_index += 1
            if self.current_index < len(self.waypoints):
                self.send_current_goal()
            else:
                self.state = 'IDLE'

    def fail_sequence(self, reason: str) -> None:
        self.generation += 1
        self.waypoints = []
        self.pending_replacement = None
        self.state = 'IDLE'
        self.wait_until_ns = None
        self.publish_status(f'Waypoint sequence stopped: {reason}')

    def safety_callback(self, msg: Bool) -> None:
        was_latched = self.safety_latched
        self.safety_latched = bool(msg.data)
        if self.safety_latched and not was_latched:
            if self.goal_handle is not None:
                self.cancel_is_user_request = True
                self.goal_handle.cancel_goal_async()
            else:
                self.fail_sequence('Safety stop latched')

    def cancel_callback(self, request, response):
        del request
        self.pending_replacement = None
        self.wait_until_ns = None
        if self.goal_handle is not None:
            self.cancel_is_user_request = True
            self.goal_handle.cancel_goal_async()
            response.success = True
            response.message = 'Active Nav2 goal cancellation requested'
        else:
            self.generation += 1
            self.waypoints = []
            self.state = 'IDLE'
            response.success = True
            response.message = 'Waypoint queue cleared'
            self.publish_status('Waypoint queue cleared')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimedWaypointExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
