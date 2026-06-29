#!/usr/bin/env python3

import math
import threading
import time
from queue import Empty, Queue

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
import serial
import tf2_ros

# Arduino state codes from Autonomous.ino
_STATE_NAMES = {0: 'RELEASED', 1: 'RUNNING', 2: 'BRAKING', 3: 'ESTOP', 4: 'WATCHDOG', 5: 'FAULT'}


class ArduinoBridgeNode(Node):
    """Bridge between ROS2 and the Arduino motor controller.

    Subscribes:
      /cmd_vel  (geometry_msgs/Twist)   → sends V command over serial

    Publishes:
      /odom     (nav_msgs/Odometry)     ← from Arduino T telemetry
      /imu      (sensor_msgs/Imu)       ← yaw + gyro from MPU6050 via Arduino
      /tf       odom → base_footprint   ← from odometry

    Serial protocol (500 000 baud, ASCII CSV):
      Pi→Arduino:  V,seq,m1,m2,m3,m4   wheel targets mm/s (left,left,right,right)
                   S,seq               normal stop
                   E,seq               emergency stop
                   C,seq               clear e-stop
                   R,seq               reset encoders + yaw
      Arduino→Pi:  READY               boot confirmation
                   T,seq,ms,e1-e4,v1-v4,yaw_cdeg,gyro_cdeg_s,state,fault
                   A,seq,code          ack (0=ok, 1=parse err, 2=estop)
    """

    def __init__(self) -> None:
        super().__init__('arduino_bridge')

        # ---- Parameters ----
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 500000)
        self.declare_parameter('wheel_separation', 0.235)   # m — tune to physical robot
        self.declare_parameter('wheel_radius', 0.04025)     # m — 80.5 mm diameter / 2
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_vel_timeout', 0.5)      # s — stop if no cmd_vel

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud_rate').value
        self._wheel_sep = self.get_parameter('wheel_separation').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._imu_frame = self.get_parameter('imu_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._cmd_timeout = self.get_parameter('cmd_vel_timeout').value

        # ---- Publishers ----
        qos = QoSProfile(depth=10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self._imu_pub = self.create_publisher(Imu, 'imu', qos)

        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---- Subscriber ----
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, qos)

        # ---- State ----
        self._seq = 0
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_odom_time = None
        self._last_cmd_time = None
        self._arduino_ready = False

        # ---- Serial ----
        try:
            self._ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as exc:
            self.get_logger().fatal(f'Cannot open serial port {port}: {exc}')
            raise SystemExit(1)

        self._recv_q: Queue = Queue()
        self._serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._serial_thread.start()

        # ---- Timers ----
        self.create_timer(0.02, self._process_queue)          # 50 Hz — drain recv queue
        self.create_timer(self._cmd_timeout, self._watchdog)  # cmd_vel watchdog

        self.get_logger().info(f'Arduino bridge started on {port} @ {baud} baud — waiting for READY')

    # ------------------------------------------------------------------
    # Serial reader thread
    # ------------------------------------------------------------------

    def _serial_reader(self) -> None:
        buf = b''
        while True:
            try:
                chunk = self._ser.read(256)
                if chunk:
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        line = line.strip(b'\r ')
                        if line:
                            self._recv_q.put(line.decode('ascii', errors='replace'))
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                time.sleep(0.1)
            except Exception as exc:
                self.get_logger().error(f'Unexpected serial error: {exc}')
                time.sleep(0.1)

    # ------------------------------------------------------------------
    # Queue processing (runs in ROS2 spin thread)
    # ------------------------------------------------------------------

    def _process_queue(self) -> None:
        for _ in range(30):
            try:
                self._parse_line(self._recv_q.get_nowait())
            except Empty:
                break

    def _parse_line(self, line: str) -> None:
        if line == 'READY':
            self._arduino_ready = True
            self.get_logger().info('Arduino READY')
            return

        parts = line.split(',')
        if not parts:
            return
        cmd = parts[0]
        if cmd == 'T':
            self._handle_telemetry(parts)
        elif cmd == 'A':
            self._handle_ack(parts)

    def _handle_ack(self, parts) -> None:
        if len(parts) >= 3 and parts[2] != '0':
            self.get_logger().warn(f'Arduino ACK error: seq={parts[1]} code={parts[2]}')

    def _handle_telemetry(self, parts) -> None:
        # T,seq,millis,e1,e2,e3,e4,v1,v2,v3,v4,yaw_cdeg,gyro_cdeg_s,state,fault
        if len(parts) < 15:
            return
        try:
            v1 = float(parts[7])   # mm/s — corrected encoder velocity, motor 1
            v2 = float(parts[8])
            v3 = float(parts[9])
            v4 = float(parts[10])
            yaw_cdeg = int(parts[11])       # centidegrees, ROS-convention (CCW positive)
            gyro_cdeg_s = int(parts[12])    # centideg/s
            state = int(parts[13])
        except (ValueError, IndexError):
            return

        now = self.get_clock().now()

        # Left = motors 1,2 (indices 0,1); Right = motors 3,4 (indices 2,3)
        left_mm_s = (v1 + v2) / 2.0
        right_mm_s = (v3 + v4) / 2.0

        linear_m_s = (left_mm_s + right_mm_s) / 2000.0
        # Use IMU gyro for angular velocity — more accurate than (right-left)/sep
        angular_rad_s = math.radians(gyro_cdeg_s / 100.0)
        # Use IMU yaw directly for heading to prevent drift
        theta_imu = math.radians(yaw_cdeg / 100.0)

        # Integrate position using IMU heading + encoder linear velocity
        if self._last_odom_time is not None:
            dt = (now - self._last_odom_time).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._theta = theta_imu
                self._x += linear_m_s * math.cos(self._theta) * dt
                self._y += linear_m_s * math.sin(self._theta) * dt

        self._last_odom_time = now

        # Quaternion from yaw only
        half = self._theta * 0.5
        qz = math.sin(half)
        qw = math.cos(half)

        self._publish_odom(now, linear_m_s, angular_rad_s, qz, qw)
        self._publish_imu(now, angular_rad_s, qz, qw)

        if state not in (0, 1):
            self.get_logger().warn(f'Arduino state: {_STATE_NAMES.get(state, state)}', throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_odom(self, now, linear: float, angular: float, qz: float, qw: float) -> None:
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular

        # Diagonal covariances (tuned conservatively)
        msg.pose.covariance[0] = 0.01   # x
        msg.pose.covariance[7] = 0.01   # y
        msg.pose.covariance[35] = 0.05  # yaw
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.05

        self._odom_pub.publish(msg)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf)

    def _publish_imu(self, now, angular_rad_s: float, qz: float, qw: float) -> None:
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._imu_frame

        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance[8] = 0.01

        msg.angular_velocity.z = angular_rad_s
        msg.angular_velocity_covariance[8] = 0.01

        # Linear acceleration not available from Arduino telemetry
        msg.linear_acceleration_covariance[0] = -1.0

        self._imu_pub.publish(msg)

    # ------------------------------------------------------------------
    # cmd_vel subscriber
    # ------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._last_cmd_time = time.monotonic()
        self._send_velocity(msg.linear.x, msg.angular.z)

    def _send_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        left = (linear_m_s - angular_rad_s * self._wheel_sep / 2.0) * 1000.0   # mm/s
        right = (linear_m_s + angular_rad_s * self._wheel_sep / 2.0) * 1000.0
        self._seq = (self._seq + 1) & 0xFFFF
        line = f'V,{self._seq},{left:.1f},{left:.1f},{right:.1f},{right:.1f}\n'
        self._serial_write(line)

    def _serial_write(self, text: str) -> None:
        try:
            self._ser.write(text.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write error: {exc}')

    # ------------------------------------------------------------------
    # Watchdog — stop motors if cmd_vel goes silent
    # ------------------------------------------------------------------

    def _watchdog(self) -> None:
        if self._last_cmd_time is None:
            return
        if time.monotonic() - self._last_cmd_time > self._cmd_timeout:
            self._send_velocity(0.0, 0.0)
            self._last_cmd_time = None  # Don't flood zeros after one stop

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.get_logger().info('Sending stop to Arduino')
        self._seq = (self._seq + 1) & 0xFFFF
        self._serial_write(f'S,{self._seq}\n')
        time.sleep(0.1)
        self._ser.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
