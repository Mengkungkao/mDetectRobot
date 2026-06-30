#!/usr/bin/env python3

import glob
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
from std_msgs.msg import UInt16
import serial
import tf2_ros

# Arduino state codes
_STATE_NAMES = {0: 'RELEASED', 1: 'RUNNING', 2: 'BRAKING',
                3: 'ESTOP', 4: 'WATCHDOG', 5: 'FAULT'}

# lidar_status values (mirrors node_lidar_ros.cpp switch cases)
_LIDAR_START = 1
_LIDAR_STOP  = 2

_READY_TIMEOUT_S    = 8.0   # how long to wait for READY after O command
_RETRY_DELAY_S      = 2.0   # initial reconnect interval
_RETRY_DELAY_MAX_S  = 30.0  # maximum reconnect interval


class ArduinoBridgeNode(Node):
    """Bridge between ROS2 and the Arduino motor controller with auto-reconnect.

    Subscribes:
      /cmd_vel  (geometry_msgs/Twist)   → V command over serial

    Publishes:
      /odom          (nav_msgs/Odometry)    ← Arduino T telemetry
      /imu           (sensor_msgs/Imu)      ← yaw + gyro from MPU6050
      /tf            odom → base_footprint  ← from odometry
      /lidar_status  (std_msgs/UInt16)      → 1=start, 2=stop LiDAR node

    Serial protocol (500 000 baud, ASCII CSV):
      Pi→Arduino:
        O,seq               open/reconnect — full reset; Arduino replies A then READY
        V,seq,m1,m2,m3,m4   wheel targets mm/s (left,left,right,right)
        S,seq               normal stop
        E,seq               emergency stop
        C,seq               clear e-stop
        R,seq               reset encoders + yaw
        X,seq               close — stop motors cleanly
      Arduino→Pi:
        READY               boot confirmation / O response
        T,seq,ms,e1-e4,v1-v4,yaw_cdeg,gyro_cdeg_s,state,fault
        A,seq,code          ack (0=ok, 1=parse err, 2=estop rejected)
    """

    def __init__(self) -> None:
        super().__init__('arduino_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('port',            '/dev/ttyUSB1')
        self.declare_parameter('baud_rate',       500000)
        self.declare_parameter('wheel_separation', 0.235)
        self.declare_parameter('wheel_radius',     0.04025)
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('base_frame',      'base_footprint')
        self.declare_parameter('imu_frame',       'imu_link')
        self.declare_parameter('publish_tf',      True)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        # Publish lidar_status 1/2 when Arduino connects/disconnects
        self.declare_parameter('control_lidar',   True)

        self._port          = self.get_parameter('port').value
        self._baud          = self.get_parameter('baud_rate').value
        self._wheel_sep     = self.get_parameter('wheel_separation').value
        self._odom_frame    = self.get_parameter('odom_frame').value
        self._base_frame    = self.get_parameter('base_frame').value
        self._imu_frame     = self.get_parameter('imu_frame').value
        self._publish_tf    = self.get_parameter('publish_tf').value
        self._cmd_timeout   = self.get_parameter('cmd_vel_timeout').value
        self._control_lidar = self.get_parameter('control_lidar').value

        # ── Publishers ────────────────────────────────────────────────────
        qos = QoSProfile(depth=10)
        self._odom_pub  = self.create_publisher(Odometry, 'odom',  qos)
        self._imu_pub   = self.create_publisher(Imu,      'imu',   qos)
        self._lidar_pub = self.create_publisher(UInt16,   'lidar_status', qos)

        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, qos)

        # ── State ─────────────────────────────────────────────────────────
        self._seq            = 0
        self._x              = 0.0
        self._y              = 0.0
        self._theta          = 0.0
        self._last_odom_time = None
        self._last_cmd_time  = None
        self._running        = True
        self._connected      = False   # True only after READY handshake

        # ── Serial ────────────────────────────────────────────────────────
        self._ser       = None          # set / cleared only by connection manager
        self._write_lock = threading.Lock()   # serialises all ser.write() calls
        self._recv_q: Queue = Queue()

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(0.02,              self._process_queue)   # 50 Hz
        self.create_timer(self._cmd_timeout, self._watchdog)

        # ── Connection manager thread ──────────────────────────────────────
        self._conn_thread = threading.Thread(
            target=self._connection_manager,
            daemon=True,
            name='serial-manager')
        self._conn_thread.start()

        self.get_logger().info(
            f'Arduino bridge starting — port={self._port} baud={self._baud}')

    # ──────────────────────────────────────────────────────────────────────
    # Connection manager  (dedicated thread — owns serial lifecycle)
    # ──────────────────────────────────────────────────────────────────────

    def _connection_manager(self) -> None:
        delay = _RETRY_DELAY_S
        while self._running:
            try:
                self._open_port()
                delay = _RETRY_DELAY_S          # reset back-off on success
                self._read_loop()               # blocks until error or shutdown
            except serial.SerialException as exc:
                if self._running:
                    self.get_logger().error(f'Serial error: {exc}')
            except Exception as exc:
                if self._running:
                    self.get_logger().error(f'Unexpected serial error: {exc}')
            finally:
                was_connected = self._connected
                self._connected = False
                self._close_port()
                if was_connected:
                    self._publish_lidar_status(_LIDAR_STOP)

            if not self._running:
                break
            self.get_logger().info(
                f'Serial disconnected — retrying in {delay:.0f}s')
            time.sleep(delay)
            delay = min(delay * 1.5, _RETRY_DELAY_MAX_S)

    def _available_serial_ports(self) -> list:
        return sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))

    def _open_port(self) -> None:
        self.get_logger().info(f'Opening serial port {self._port}')
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.1)
        except serial.SerialException as exc:
            available = self._available_serial_ports()
            hint = f'Available ports: {available}' if available else 'No serial ports found.'
            self.get_logger().error(
                f'Cannot open {self._port} — {hint}\n'
                f'Relaunch with: arduino_port:=<correct port>')
            raise
        ser.reset_input_buffer()

        # Arduino may already be running (reconnect case) so don't wait long
        # for the boot READY — we'll get one anyway after the O command.
        boot_ready = self._wait_for_ready(ser, timeout=2.0)
        if boot_ready:
            self.get_logger().debug('Received boot READY before O command')

        # Hand the port to the writer before sending O
        with self._write_lock:
            self._ser = ser

        # Send O to reset Arduino state and solicit a fresh READY
        self._seq = (self._seq + 1) & 0xFFFF
        self._write_raw(f'O,{self._seq}\n')

        # Wait for the READY that O triggers (consume any A,... ACK in between)
        if not self._wait_for_ready(ser, timeout=_READY_TIMEOUT_S):
            with self._write_lock:
                self._ser = None
            ser.close()
            raise serial.SerialException(
                f'Arduino on {self._port} did not send READY after O command')

        self._connected = True
        self.get_logger().info(f'Arduino connected on {self._port}')
        self._publish_lidar_status(_LIDAR_START)

    def _close_port(self) -> None:
        with self._write_lock:
            ser      = self._ser
            self._ser = None

        if ser is None:
            return

        # Send X so Arduino stops motors before we drop the line
        try:
            self._seq = (self._seq + 1) & 0xFFFF
            ser.write(f'X,{self._seq}\n'.encode('ascii'))
            time.sleep(0.15)
        except Exception:
            pass

        try:
            ser.close()
            self.get_logger().info(f'Serial port {self._port} closed')
        except Exception:
            pass

    def _wait_for_ready(self, ser: serial.Serial, timeout: float) -> bool:
        """Read bytes from ser directly until READY is seen or timeout expires."""
        buf = b''
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                chunk = ser.read(256)
            except serial.SerialException:
                return False
            if chunk:
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.strip(b'\r ').decode('ascii', errors='replace')
                    if text == 'READY':
                        return True
        return False

    def _read_loop(self) -> None:
        """Read serial bytes and push complete lines onto the queue."""
        buf = b''
        while self._running and self._connected:
            ser = self._ser
            if ser is None:
                break
            try:
                chunk = ser.read(256)
            except serial.SerialException:
                self._connected = False
                break
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.strip(b'\r ')
                if line:
                    self._recv_q.put(line.decode('ascii', errors='replace'))

    # ──────────────────────────────────────────────────────────────────────
    # Queue processing  (ROS spin thread — 50 Hz)
    # ──────────────────────────────────────────────────────────────────────

    def _process_queue(self) -> None:
        for _ in range(30):
            try:
                self._parse_line(self._recv_q.get_nowait())
            except Empty:
                break

    def _parse_line(self, line: str) -> None:
        if line == 'READY':
            # Spurious READY means Arduino rebooted while we were running.
            # Signal the read loop to exit so the connection manager reconnects.
            self.get_logger().warn(
                'Unexpected READY from Arduino — triggering reconnect')
            self._connected = False
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
            self.get_logger().warn(
                f'Arduino ACK error: seq={parts[1]} code={parts[2]}')

    def _handle_telemetry(self, parts) -> None:
        # T,seq,millis,e1,e2,e3,e4,v1,v2,v3,v4,yaw_cdeg,gyro_cdeg_s,state,fault
        if len(parts) < 15:
            return
        try:
            v1          = float(parts[7])
            v2          = float(parts[8])
            v3          = float(parts[9])
            v4          = float(parts[10])
            yaw_cdeg    = int(parts[11])
            gyro_cdeg_s = int(parts[12])
            state       = int(parts[13])
        except (ValueError, IndexError):
            return

        now = self.get_clock().now()

        # Left = motors 1,2 (indices 0,1); Right = motors 3,4 (indices 2,3)
        left_mm_s  = (v1 + v2) / 2.0
        right_mm_s = (v3 + v4) / 2.0

        linear_m_s    = (left_mm_s + right_mm_s) / 2000.0
        angular_rad_s = math.radians(gyro_cdeg_s / 100.0)
        theta_imu     = math.radians(yaw_cdeg    / 100.0)

        if self._last_odom_time is not None:
            dt = (now - self._last_odom_time).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._theta = theta_imu
                self._x += linear_m_s * math.cos(self._theta) * dt
                self._y += linear_m_s * math.sin(self._theta) * dt

        self._last_odom_time = now

        half = self._theta * 0.5
        qz   = math.sin(half)
        qw   = math.cos(half)

        self._publish_odom(now, linear_m_s, angular_rad_s, qz, qw)
        self._publish_imu(now, angular_rad_s, qz, qw)

        if state not in (0, 1):
            self.get_logger().warn(
                f'Arduino state: {_STATE_NAMES.get(state, state)}',
                throttle_duration_sec=2.0)

    # ──────────────────────────────────────────────────────────────────────
    # Publishers
    # ──────────────────────────────────────────────────────────────────────

    def _publish_odom(self, now, linear: float, angular: float,
                      qz: float, qw: float) -> None:
        msg = Odometry()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id  = self._base_frame

        msg.pose.pose.position.x    = self._x
        msg.pose.pose.position.y    = self._y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x  = linear
        msg.twist.twist.angular.z = angular

        msg.pose.covariance[0]   = 0.01
        msg.pose.covariance[7]   = 0.01
        msg.pose.covariance[35]  = 0.05
        msg.twist.covariance[0]  = 0.01
        msg.twist.covariance[35] = 0.05

        self._odom_pub.publish(msg)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp         = now.to_msg()
            tf.header.frame_id      = self._odom_frame
            tf.child_frame_id       = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self._tf_broadcaster.sendTransform(tf)

    def _publish_imu(self, now, angular_rad_s: float,
                     qz: float, qw: float) -> None:
        msg = Imu()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = self._imu_frame

        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance[8]      = 0.01

        msg.angular_velocity.z             = angular_rad_s
        msg.angular_velocity_covariance[8] = 0.01

        msg.linear_acceleration_covariance[0] = -1.0   # not available
        self._imu_pub.publish(msg)

    def _publish_lidar_status(self, status: int) -> None:
        if not self._control_lidar:
            return
        msg      = UInt16()
        msg.data = status
        self._lidar_pub.publish(msg)
        label = 'START' if status == _LIDAR_START else 'STOP'
        self.get_logger().info(f'LiDAR control: {label}')

    # ──────────────────────────────────────────────────────────────────────
    # Serial write helpers
    # ──────────────────────────────────────────────────────────────────────

    def _write_raw(self, text: str) -> None:
        """Write text while holding the write lock. Caller must not hold it."""
        with self._write_lock:
            ser = self._ser
            if ser is None:
                return
            try:
                ser.write(text.encode('ascii'))
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial write error: {exc}')
                self._connected = False

    def _serial_write(self, text: str) -> None:
        """Drop silently when not connected."""
        if self._connected:
            self._write_raw(text)

    # ──────────────────────────────────────────────────────────────────────
    # cmd_vel subscriber
    # ──────────────────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._last_cmd_time = time.monotonic()
        if self._connected:
            self._send_velocity(msg.linear.x, msg.angular.z)

    def _send_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        left  = (linear_m_s - angular_rad_s * self._wheel_sep / 2.0) * 1000.0
        right = (linear_m_s + angular_rad_s * self._wheel_sep / 2.0) * 1000.0
        self._seq = (self._seq + 1) & 0xFFFF
        self._serial_write(
            f'V,{self._seq},{left:.1f},{left:.1f},{right:.1f},{right:.1f}\n')

    # ──────────────────────────────────────────────────────────────────────
    # Watchdog — stop motors when cmd_vel goes silent
    # ──────────────────────────────────────────────────────────────────────

    def _watchdog(self) -> None:
        if self._last_cmd_time is None:
            return
        if time.monotonic() - self._last_cmd_time > self._cmd_timeout:
            if self._connected:
                self._send_velocity(0.0, 0.0)
            self._last_cmd_time = None   # one stop per silence period

    # ──────────────────────────────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down Arduino bridge')

        # Send a graceful stop while still connected
        if self._connected:
            self._seq = (self._seq + 1) & 0xFFFF
            self._serial_write(f'S,{self._seq}\n')
            time.sleep(0.1)

        self._running   = False
        self._connected = False
        self._close_port()
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
