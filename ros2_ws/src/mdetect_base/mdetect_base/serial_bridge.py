#!/usr/bin/env python3
"""ROS 2 to Arduino serial bridge for the mDetect four-motor robot."""

import json
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - shown on the robot if dependency is missing
    serial = None
    SerialException = Exception
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SerialBridge(Node):
    """Convert safe cmd_vel commands to four wheel targets and publish telemetry."""

    def __init__(self) -> None:
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 500000)
        self.declare_parameter('reconnect_period', 2.0)
        self.declare_parameter('command_rate', 50.0)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('telemetry_timeout', 0.5)
        self.declare_parameter('counts_per_rev', 4320.0)
        self.declare_parameter('wheel_diameter', 0.0805)
        self.declare_parameter('track_width', 0.190)
        self.declare_parameter('max_wheel_speed', 0.30)
        self.declare_parameter('drive_mode', 'skid')
        self.declare_parameter('linear_sign', 1.0)
        self.declare_parameter('angular_sign', 1.0)
        self.declare_parameter('publish_odom_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter(
            'joint_names',
            ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint', 'wheel_4_joint'],
        )
        self.declare_parameter('reset_on_connect', False)
        self.declare_parameter('normal_brake_hold_ms', 1000)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.reconnect_period = float(self.get_parameter('reconnect_period').value)
        self.command_rate = max(float(self.get_parameter('command_rate').value), 1.0)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout').value)
        self.counts_per_rev = float(self.get_parameter('counts_per_rev').value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.drive_mode = str(self.get_parameter('drive_mode').value).lower()
        self.linear_sign = float(self.get_parameter('linear_sign').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.reset_on_connect = bool(self.get_parameter('reset_on_connect').value)
        self.normal_brake_hold_ms = int(self.get_parameter('normal_brake_hold_ms').value)

        if len(self.joint_names) != 4:
            raise ValueError('joint_names must contain exactly four names')
        if self.drive_mode != 'skid':
            self.get_logger().warning(
                "Only 'skid' mode is enabled in this release; using skid mixing."
            )
            self.drive_mode = 'skid'

        self.mm_per_count = math.pi * self.wheel_diameter * 1000.0 / self.counts_per_rev
        self.serial_port = None
        self.rx_buffer = bytearray()
        self.last_connect_attempt_ns = 0
        self.last_telemetry_ns: Optional[int] = None
        self.last_cmd_ns: Optional[int] = None
        self.latest_cmd = Twist()
        self.sequence = 0
        self.connected = False
        self.estop_latched = False
        self.controller_state = 0
        self.fault_code = 0
        self.last_ack = ''

        self.last_counts: Optional[List[int]] = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_stamp_ns: Optional[int] = None

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_safe', self.cmd_callback, 20)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 20)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 20)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 20)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/wheel/encoder_counts', 20)
        self.speed_pub = self.create_publisher(Float32MultiArray, '/wheel/speeds', 20)
        self.status_pub = self.create_publisher(String, '/base/status', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None

        self.create_service(Trigger, '/base/emergency_stop', self.estop_callback)
        self.create_service(Trigger, '/base/clear_emergency_stop', self.clear_estop_callback)
        self.create_service(Trigger, '/base/brake', self.brake_callback)
        self.create_service(Trigger, '/base/reset_odometry', self.reset_callback)

        self.io_timer = self.create_timer(0.005, self.io_cycle)
        self.command_timer = self.create_timer(1.0 / self.command_rate, self.send_velocity_cycle)
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        if SERIAL_IMPORT_ERROR is not None:
            self.get_logger().error(
                f'pyserial is not installed: {SERIAL_IMPORT_ERROR}. '
                'Install with: sudo apt install python3-serial'
            )
        self.get_logger().info(
            f'Serial bridge configured for {self.port} at {self.baud} baud'
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_ns = self.now_ns()

    def next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) & 0xFFFF
        return self.sequence

    def connect_serial(self) -> None:
        if serial is None or self.serial_port is not None:
            return
        now = self.now_ns()
        if now - self.last_connect_attempt_ns < int(self.reconnect_period * 1e9):
            return
        self.last_connect_attempt_ns = now

        try:
            port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.0,
                write_timeout=0.05,
            )
            port.reset_input_buffer()
            port.reset_output_buffer()
            self.serial_port = port
            self.connected = True
            self.rx_buffer.clear()
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            if self.reset_on_connect:
                self.write_command(f'R,{self.next_sequence()}')
        except (SerialException, OSError) as exc:
            self.connected = False
            self.get_logger().warning(f'Cannot open {self.port}: {exc}')

    def disconnect_serial(self, reason: str) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        if self.connected:
            self.get_logger().error(f'Arduino serial disconnected: {reason}')
        self.connected = False
        self.rx_buffer.clear()

    def write_command(self, command: str) -> bool:
        if self.serial_port is None:
            return False
        try:
            self.serial_port.write((command + '\n').encode('ascii'))
            return True
        except (SerialException, OSError) as exc:
            self.disconnect_serial(str(exc))
            return False

    def io_cycle(self) -> None:
        if self.serial_port is None:
            self.connect_serial()
            return

        try:
            available = self.serial_port.in_waiting
            if available:
                self.rx_buffer.extend(self.serial_port.read(available))
        except (SerialException, OSError) as exc:
            self.disconnect_serial(str(exc))
            return

        while b'\n' in self.rx_buffer:
            raw_line, _, remainder = self.rx_buffer.partition(b'\n')
            self.rx_buffer = bytearray(remainder)
            line = raw_line.decode('ascii', errors='ignore').strip('\r\0 ')
            if line:
                self.handle_line(line)

        if len(self.rx_buffer) > 1024:
            self.get_logger().warning('Discarding oversized serial receive buffer')
            self.rx_buffer.clear()

    def handle_line(self, line: str) -> None:
        if line == 'READY':
            self.get_logger().info('Arduino low-level controller is ready')
            return
        if line.startswith('A,'):
            self.last_ack = line
            return
        if not line.startswith('T,'):
            self.get_logger().debug(f'Arduino: {line}')
            return

        fields = line.split(',')
        if len(fields) != 15:
            self.get_logger().warning(f'Invalid telemetry field count: {line}')
            return

        try:
            sequence = int(fields[1])
            arduino_ms = int(fields[2])
            counts = [int(value) for value in fields[3:7]]
            speeds_mm_s = [float(value) for value in fields[7:11]]
            yaw_deg = int(fields[11]) / 100.0
            gyro_deg_s = int(fields[12]) / 100.0
            state = int(fields[13])
            fault = int(fields[14])
        except (ValueError, IndexError) as exc:
            self.get_logger().warning(f'Cannot parse telemetry: {exc}: {line}')
            return

        self.last_telemetry_ns = self.now_ns()
        self.controller_state = state
        self.fault_code = fault
        self.estop_latched = bool(fault & 0x02) or state == 3
        self.publish_telemetry(sequence, arduino_ms, counts, speeds_mm_s, yaw_deg, gyro_deg_s)

    def publish_telemetry(
        self,
        sequence: int,
        arduino_ms: int,
        counts: List[int],
        speeds_mm_s: List[float],
        yaw_deg: float,
        gyro_deg_s: float,
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        encoder_msg = Int32MultiArray()
        encoder_msg.data = counts
        self.encoder_pub.publish(encoder_msg)

        speed_msg = Float32MultiArray()
        speed_msg.data = [value / 1000.0 for value in speeds_mm_s]
        self.speed_pub.publish(speed_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = self.joint_names
        joint_msg.position = [count * 2.0 * math.pi / self.counts_per_rev for count in counts]
        joint_msg.velocity = [
            (value / 1000.0) / (self.wheel_diameter * 0.5)
            for value in speeds_mm_s
        ]
        self.joint_pub.publish(joint_msg)

        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.08,
        ]
        imu_msg.angular_velocity.z = math.radians(gyro_deg_s)
        imu_msg.angular_velocity_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.03,
        ]
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

        self.update_and_publish_odometry(counts, stamp)

        status = String()
        status.data = json.dumps({
            'connected': self.connected,
            'sequence': sequence,
            'arduino_ms': arduino_ms,
            'state': self.controller_state,
            'fault': self.fault_code,
            'estop': self.estop_latched,
        })
        self.status_pub.publish(status)

    def update_and_publish_odometry(self, counts: List[int], stamp) -> None:
        now_ns = self.now_ns()
        if self.last_counts is None:
            self.last_counts = list(counts)
            self.last_odom_stamp_ns = now_ns
            return

        delta = [counts[i] - self.last_counts[i] for i in range(4)]
        self.last_counts = list(counts)
        dt = max((now_ns - (self.last_odom_stamp_ns or now_ns)) / 1e9, 1e-3)
        self.last_odom_stamp_ns = now_ns

        # Robot wiring groups motors 1 and 4 on the right, and 2 and 3 on the left.
        right_distance = ((delta[0] + delta[3]) * 0.5) * self.mm_per_count / 1000.0
        left_distance = ((delta[1] + delta[2]) * 0.5) * self.mm_per_count / 1000.0
        distance = 0.5 * (right_distance + left_distance)
        dtheta = (right_distance - left_distance) / self.track_width

        heading_mid = self.odom_yaw + 0.5 * dtheta
        self.odom_x += distance * math.cos(heading_mid)
        self.odom_y += distance * math.sin(heading_mid)
        self.odom_yaw = math.atan2(
            math.sin(self.odom_yaw + dtheta), math.cos(self.odom_yaw + dtheta)
        )

        qx, qy, qz, qw = yaw_to_quaternion(self.odom_yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = distance / dt
        odom.twist.twist.angular.z = dtheta / dt
        odom.pose.covariance = [
            0.03, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.03, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
        ]
        odom.twist.covariance = list(odom.pose.covariance)
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.odom_x
            transform.transform.translation.y = self.odom_y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def mix_wheel_speeds(self, command: Twist) -> List[float]:
        vx = float(command.linear.x) * self.linear_sign
        wz = float(command.angular.z) * self.angular_sign
        half_track = 0.5 * self.track_width

        right = vx + half_track * wz
        left = vx - half_track * wz
        wheel_speeds = [right, left, left, right]

        peak = max(abs(value) for value in wheel_speeds)
        if peak > self.max_wheel_speed > 0.0:
            scale = self.max_wheel_speed / peak
            wheel_speeds = [value * scale for value in wheel_speeds]
        return wheel_speeds

    def send_velocity_cycle(self) -> None:
        if self.serial_port is None:
            return

        now = self.now_ns()
        stale = (
            self.last_cmd_ns is None or
            now - self.last_cmd_ns > int(self.command_timeout * 1e9)
        )
        command = Twist() if stale or self.estop_latched else self.latest_cmd
        wheel_speeds = self.mix_wheel_speeds(command)
        mm_s = [int(round(value * 1000.0)) for value in wheel_speeds]
        seq = self.next_sequence()
        self.write_command(
            f'V,{seq},{mm_s[0]},{mm_s[1]},{mm_s[2]},{mm_s[3]}'
        )

    def estop_callback(self, request, response):
        del request
        seq = self.next_sequence()
        ok = self.write_command(f'E,{seq}')
        self.estop_latched = True
        response.success = ok
        response.message = 'Emergency stop sent and latched' if ok else 'Arduino not connected'
        return response

    def clear_estop_callback(self, request, response):
        del request
        seq = self.next_sequence()
        ok = self.write_command(f'C,{seq}')
        if ok:
            self.estop_latched = False
        response.success = ok
        response.message = 'Emergency stop cleared' if ok else 'Arduino not connected'
        return response

    def brake_callback(self, request, response):
        del request
        seq = self.next_sequence()
        ok = self.write_command(f'B,{seq},{self.normal_brake_hold_ms}')
        response.success = ok
        response.message = 'Brake command sent' if ok else 'Arduino not connected'
        return response

    def reset_callback(self, request, response):
        del request
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_counts = None
        self.last_odom_stamp_ns = None
        seq = self.next_sequence()
        ok = self.write_command(f'R,{seq}')
        response.success = ok
        response.message = (
            'Arduino sensors and local wheel odometry reset' if ok else 'Arduino not connected'
        )
        return response

    def publish_diagnostics(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'mdetect/arduino_serial_bridge'
        status.hardware_id = self.port

        now = self.now_ns()
        telemetry_age = float('inf') if self.last_telemetry_ns is None else (
            now - self.last_telemetry_ns
        ) / 1e9

        if not self.connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Serial port disconnected'
        elif telemetry_age > self.telemetry_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Arduino telemetry timeout'
        elif self.estop_latched:
            status.level = DiagnosticStatus.WARN
            status.message = 'Emergency stop latched'
        elif self.fault_code:
            status.level = DiagnosticStatus.WARN
            status.message = f'Arduino fault code {self.fault_code}'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Arduino bridge operational'

        status.values = [
            KeyValue(key='connected', value=str(self.connected)),
            KeyValue(key='telemetry_age_s', value=f'{telemetry_age:.3f}'),
            KeyValue(key='controller_state', value=str(self.controller_state)),
            KeyValue(key='fault_code', value=str(self.fault_code)),
            KeyValue(key='last_ack', value=self.last_ack),
        ]
        msg.status.append(status)
        self.diag_pub.publish(msg)

    def destroy_node(self):
        if self.serial_port is not None:
            try:
                self.write_command(f'S,{self.next_sequence()}')
                self.serial_port.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
