#!/usr/bin/env python3
"""
Pygame robot waypoint controller + Arduino odometry + COIN-D6 LiDAR map.

Arduino side:
- Serial.begin(500000)
- Command format: (X_mm,Y_mm,wait_seconds):
- Feedback pose format: X>Y>Heading
- The left panel now shows parsed pose plus more raw Arduino serial lines.

COIN-D6 LiDAR side:
- Port: /dev/ttyUSB1
- Baud: 230400
- Start command: AA 55 F0 0F
- Stop command:  AA 55 F5 0A

Keys:
- C = clear live LiDAR scan only
- M = clear fixed occupancy map
- P = plan a safe obstacle-avoiding route from current robot position to target X/Y
- S = send emergency STOP to Arduino
- U = Reset All: reset Arduino Uno through USB DTR and clear local map/route state
- V = switch LiDAR odometry verification ON/OFF
- B = switch between dark and white background
- Q/E = rotate LiDAR heading offset -/+ 5 deg
- R = reset LiDAR heading offset and clear fixed map to 0 deg
- UP = zoom in / reduce range
- DOWN = zoom out / increase range
- ESC = quit

Window:
- The Pygame window is resizable. The LiDAR map expands/shrinks with the window.

Mapping/path planning:
- Builds a lightweight occupancy-grid SLAM map from Arduino odometry + COIN-D6 LiDAR.
- Uses LiDAR scan matching to create a corrected SLAM pose for mapping and drawing when confidence is good.
- Verifies encoder X/Y and MPU6050 yaw by comparing them with the LiDAR-matched pose.
- Creates an inflated cost map using a 200 mm robot radius from the centre, so the full robot frame avoids obstacles.
- Uses A* to plan from current robot position to target X/Y, then converts the route into 1 to 10 ordered waypoints for Arduino.

Safety stop:
- If COIN-D6 detects an object in front of the robot body with less than 250 mm clearance,
  Python sends STOP: to Arduino.
"""

import math
import re
import time
import heapq
from collections import deque

import pygame
import serial

# =========================
# SERIAL PORT SETUP
# =========================
# Change these two ports to match your computer/Raspberry Pi.
# Raspberry Pi / Ubuntu serial setup
# Arduino and COIN-D6 LiDAR are on separate USB serial ports.
ARDUINO_PORT = "/dev/ttyUSB0"
ARDUINO_BAUD = 500000
ARDUINO_TIMEOUT = 0.0

LIDAR_PORT = "/dev/ttyUSB1"
LIDAR_BAUD = 230400
LIDAR_TIMEOUT = 0.0

# COIN-D6 commands and packet header
START_CMD = bytes([0xAA, 0x55, 0xF0, 0x0F])
STOP_CMD = bytes([0xAA, 0x55, 0xF5, 0x0A])
PACKET_HEADER = bytes([0xAA, 0x55])

# =========================
# OPEN ARDUINO SERIAL
# =========================
arduino = None
try:
    arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=ARDUINO_TIMEOUT)
    time.sleep(2.0)
    arduino.reset_input_buffer()
    print(f"Arduino connected on {ARDUINO_PORT} at {ARDUINO_BAUD} baud")
except Exception as e:
    print("Arduino serial connection failed:", e)

# =========================
# OPEN COIN-D6 LiDAR SERIAL
# =========================
lidar = None
try:
    lidar = serial.Serial(
        port=LIDAR_PORT,
        baudrate=LIDAR_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=LIDAR_TIMEOUT,
    )
    time.sleep(0.3)
    lidar.reset_input_buffer()
    lidar.write(START_CMD)
    print(f"COIN-D6 LiDAR connected on {LIDAR_PORT} at {LIDAR_BAUD} baud")
    print("LiDAR start command sent: AA 55 F0 0F")
except Exception as e:
    print("COIN-D6 LiDAR connection failed:", e)

# =========================
# PYGAME SETUP
# =========================
pygame.init()

MIN_WIDTH, MIN_HEIGHT = 760, 430
WINDOWED_WIDTH, WINDOWED_HEIGHT = 900, 500
START_FULLSCREEN = True

# Start the controller in fullscreen as soon as the Pygame window opens.
# Press F11 to switch between fullscreen and a normal resizable window.
fullscreen = START_FULLSCREEN
if fullscreen:
    display_info = pygame.display.Info()
    WIDTH = max(MIN_WIDTH, display_info.current_w)
    HEIGHT = max(MIN_HEIGHT, display_info.current_h)
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN)
else:
    WIDTH, HEIGHT = WINDOWED_WIDTH, WINDOWED_HEIGHT
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)

pygame.display.set_caption("Robot Controller + COIN-D6 LiDAR Map - Fullscreen")

font = pygame.font.SysFont("Arial", 22)
small_font = pygame.font.SysFont("Arial", 17)
tiny_font = pygame.font.SysFont("Arial", 14)
clock = pygame.time.Clock()

last_message = "Waiting for command..."
serial_log = deque(maxlen=30)
arduino_rx_count = 0
last_arduino_raw = ""

# =========================
# BACKGROUND / THEME SETUP
# =========================
# False = original dark background, True = white background.
# Press B or use the on-screen button to switch while the program is running.
WHITE_BACKGROUND = False


def background_color():
    return (245, 245, 245) if WHITE_BACKGROUND else (25, 25, 25)


def main_text_color():
    return (20, 20, 20) if WHITE_BACKGROUND else (255, 255, 255)


def muted_text_color():
    return (85, 85, 85) if WHITE_BACKGROUND else (180, 180, 180)


def secondary_text_color():
    return (55, 55, 55) if WHITE_BACKGROUND else (220, 220, 220)


def highlight_text_color():
    return (0, 110, 65) if WHITE_BACKGROUND else (0, 255, 120)


def input_text_color():
    return (20, 20, 20) if WHITE_BACKGROUND else (255, 255, 255)


def button_fill_color():
    return (232, 232, 232) if WHITE_BACKGROUND else (55, 55, 55)


def button_border_color():
    return (80, 80, 80) if WHITE_BACKGROUND else (200, 200, 200)


def button_text_color():
    return (20, 20, 20) if WHITE_BACKGROUND else (255, 255, 255)


def status_panel_color():
    return (238, 238, 238) if WHITE_BACKGROUND else (35, 35, 35)


def status_border_color():
    return (120, 120, 120)


def map_panel_color():
    return (255, 255, 255) if WHITE_BACKGROUND else (15, 15, 15)


def map_plot_border_color():
    return (185, 205, 190) if WHITE_BACKGROUND else (35, 65, 45)


def map_grid_color():
    return (215, 215, 215) if WHITE_BACKGROUND else (50, 50, 50)


def map_axis_color():
    return (190, 200, 200) if WHITE_BACKGROUND else (55, 70, 70)


def map_frame_box_color():
    return (246, 250, 246) if WHITE_BACKGROUND else (22, 32, 24)


def map_frame_box_border_color():
    return (125, 170, 135) if WHITE_BACKGROUND else (70, 120, 80)


def robot_marker_color():
    return (20, 20, 20) if WHITE_BACKGROUND else (255, 255, 255)


def robot_frame_color():
    return (90, 90, 90) if WHITE_BACKGROUND else (230, 230, 230)


def serial_panel_color():
    return (255, 255, 255) if WHITE_BACKGROUND else (18, 18, 18)


def serial_panel_border_color():
    return (170, 170, 170) if WHITE_BACKGROUND else (90, 90, 90)


def serial_ok_color():
    return (0, 125, 70) if WHITE_BACKGROUND else (0, 255, 120)


def serial_warning_color():
    return (170, 95, 0) if WHITE_BACKGROUND else (255, 180, 70)


def toggle_background_theme():
    """Toggle the main Pygame background between dark and white."""
    global WHITE_BACKGROUND, last_message
    WHITE_BACKGROUND = not WHITE_BACKGROUND
    last_message = "White background ON" if WHITE_BACKGROUND else "Dark background ON"


def set_background_white():
    """Force the Pygame screen background to white."""
    global WHITE_BACKGROUND, last_message
    WHITE_BACKGROUND = True
    last_message = "White background ON"


def set_background_dark():
    """Return the Pygame screen background to the original dark mode."""
    global WHITE_BACKGROUND, last_message
    WHITE_BACKGROUND = False
    last_message = "Dark background ON"


def toggle_fullscreen():
    """Switch between fullscreen and a normal resizable window."""
    global screen, WIDTH, HEIGHT, fullscreen, WINDOWED_WIDTH, WINDOWED_HEIGHT, last_message

    fullscreen = not fullscreen

    if fullscreen:
        # Save the current window size before entering fullscreen.
        WINDOWED_WIDTH, WINDOWED_HEIGHT = WIDTH, HEIGHT
        display_info = pygame.display.Info()
        WIDTH = max(MIN_WIDTH, display_info.current_w)
        HEIGHT = max(MIN_HEIGHT, display_info.current_h)
        screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN)
        last_message = "Fullscreen ON - press F11 to return to window mode"
    else:
        WIDTH = max(MIN_WIDTH, WINDOWED_WIDTH)
        HEIGHT = max(MIN_HEIGHT, WINDOWED_HEIGHT)
        screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
        last_message = "Window mode ON - press F11 for fullscreen"

    update_layout()

# Arduino pose from X>Y>Heading
robot_x = 0.0
robot_y = 0.0
robot_heading = 0.0
last_arduino_time = 0.0
arduino_line_buffer = bytearray()

# LiDAR data
lidar_buffer = bytearray()
latest_lidar_points = {}   # angle_bin: (angle_deg, distance_mm, intensity, timestamp)
lidar_packet_count = 0
last_lidar_time = 0.0
max_range_m = 4.0

# LiDAR heading calibration.
# Use this when the physical COIN-D6 LiDAR zero-degree direction does not point
# exactly to the robot front. Positive offset rotates the LiDAR points clockwise.
LIDAR_HEADING_OFFSET_DEG = 0.0
LIDAR_OFFSET_STEP_DEG = 5.0

# SLAM pose layer.
# Arduino still provides the raw encoder X/Y and MPU6050 yaw. The LiDAR scan matcher
# can provide a corrected pose; when confidence is good, this corrected pose is used
# for mapping/drawing so the map stays fixed and the robot moves inside it.
SLAM_POSE_ENABLED = True
SLAM_CONFIDENCE_TO_APPLY = 0.65
SLAM_BLEND_ALPHA = 0.55
slam_x = 0.0
slam_y = 0.0
slam_heading = 0.0
slam_confidence = 0.0
slam_status = "Arduino pose only"

# Fixed occupancy-grid map / cost map / path planning
# This is a lightweight LiDAR-assisted SLAM-style map. The first map is created
# from Arduino odometry; once enough map exists, scan matching is used to reduce
# drift and keep the room map fixed in world coordinates.
MAP_RES_MM = 50                         # each grid cell = 50 mm
MAP_FREE_DECAY = 1
MAP_OCC_INC = 4
MAP_FREE_DEC = 1
MAP_MIN = -20
MAP_MAX = 100
OCCUPIED_THRESHOLD = 18
COST_OBSTACLE = 255

# Robot frame / collision clearance.
# The robot is treated as a circle with 200 mm radius from the centre point.
# The planner inflates all obstacles by the robot radius plus a small margin,
# so the centre path stays far enough away for the full robot body to pass.
ROBOT_RADIUS_MM = 200
ROBOT_CLEARANCE_MARGIN_MM = 50
INFLATION_RADIUS_MM = ROBOT_RADIUS_MM + ROBOT_CLEARANCE_MARGIN_MM
INFLATION_CELLS = max(1, int(math.ceil(INFLATION_RADIUS_MM / MAP_RES_MM)))
ROBOT_RADIUS_CELLS = max(1, int(math.ceil(ROBOT_RADIUS_MM / MAP_RES_MM)))
MAX_RAY_RANGE_MM = 4000
MAPPING_ENABLED = True

# Fixed map display. The green map view is centred on a fixed world origin, so
# the room/walls stay still and the robot marker moves inside the map.
MAP_VIEW_FIXED = True
MAP_VIEW_CENTER_X = 0.0
MAP_VIEW_CENTER_Y = 0.0
MAP_DRAW_ONLY_INSIDE_BOX = True

# Mapping gate for the first planned route point.
# After the first movement/rotation reaches the first scan point, mapping restarts.
# The map update is paused during that first movement, then the old temporary map
# is cleared and scanning starts again from the first reached coordinate.
MAPPING_WAIT_FOR_FIRST_ROUTE_POINT = False
MAPPING_FIRST_ROUTE_POINT = None
MAPPING_FIRST_POINT_TOLERANCE_MM = 160
MAPPING_RESTART_CLEAR_MAP = True
mapping_status = "mapping live"

occupancy_grid = {}                     # (gx, gy) -> log-odds-like score
cost_grid = {}                          # (gx, gy) -> 0..255
planned_path = []                        # Full planned/smoothed path for drawing
path_waypoints = []                      # 1..10 ordered command route points for Arduino
MAX_ROUTE_POINTS = 10                    # Arduino waypoint limit
MIN_ROUTE_POINTS = 1
ROUTE_POINT_SPACING_MM = 300             # preferred spacing before reducing to 10 points
planning_message = "No path planned"

# Navigation mode: manual Plan Route + Send Path.
# The robot follows Arduino waypoint commands using the fixed SLAM map and front safety stop.

# LiDAR odometry verification / scan matching.
# This does not blindly overwrite the Arduino odometry. It estimates a LiDAR-based
# pose correction by matching the current LiDAR scan to the fixed occupancy map,
# then shows the difference between:
#   Arduino encoder X/Y + MPU6050 yaw
#   LiDAR-matched X/Y/yaw
LIDAR_ODOM_ENABLED = True
LIDAR_ODOM_INTERVAL = 0.25
LIDAR_ODOM_MAX_SCAN_AGE = 0.45
LIDAR_ODOM_MIN_POINTS = 35
LIDAR_ODOM_MIN_MAP_CELLS = 25
LIDAR_ODOM_SEARCH_XY_MM = 180
LIDAR_ODOM_SEARCH_XY_STEP_MM = 60
LIDAR_ODOM_SEARCH_YAW_DEG = 10
LIDAR_ODOM_SEARCH_YAW_STEP_DEG = 2.5
LIDAR_ODOM_MAX_ACCEPT_XY_MM = 220
LIDAR_ODOM_MAX_ACCEPT_YAW_DEG = 12
LIDAR_ODOM_GOOD_SCORE = 0.23

lidar_odom_x = 0.0
lidar_odom_y = 0.0
lidar_odom_heading = 0.0
lidar_odom_error_x = 0.0
lidar_odom_error_y = 0.0
lidar_odom_error_heading = 0.0
lidar_odom_error_dist = 0.0
lidar_odom_confidence = 0.0
lidar_odom_match_score = 0.0
lidar_odom_points_used = 0
lidar_odom_status = "waiting for map"
last_lidar_odom_time = 0.0


# Front obstacle safety stop
AUTO_STOP_ENABLED = True
# Stop when an obstacle is within 250 mm of the front of the robot frame.
# Since the robot radius is 200 mm from centre, the LiDAR-centre trigger distance is 450 mm.
FRONT_STOP_CLEARANCE_MM = 250
FRONT_STOP_DISTANCE_MM = ROBOT_RADIUS_MM + FRONT_STOP_CLEARANCE_MM
FRONT_STOP_LATERAL_MARGIN_MM = 60
FRONT_CONE_HALF_ANGLE_DEG = 18
STOP_REPEAT_SECONDS = 0.30
obstacle_detected = False
front_obstacle_distance_mm = None          # distance from LiDAR/robot centre
front_obstacle_clearance_mm = None         # estimated clearance from robot frame
last_stop_command_time = 0.0

# Stored waypoint queue: [(x_mm, y_mm, wait_sec), ...]
waypoints = []

# =========================
# INPUT BOX CLASS
# =========================
class InputBox:
    def __init__(self, x, y, w, h, label, text=""):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.text = text
        self.active = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)

        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif event.key == pygame.K_RETURN:
                self.active = False
            else:
                if event.unicode.isdigit() or event.unicode == "-":
                    if event.unicode == "-" and len(self.text) > 0:
                        return
                    self.text += event.unicode

    def value_int(self, default=0):
        try:
            if self.text.strip() in ["", "-"]:
                return default
            return int(self.text)
        except ValueError:
            return default

    def draw(self, surface):
        label_surface = tiny_font.render(self.label, True, secondary_text_color())
        surface.blit(label_surface, (self.rect.x, self.rect.y - 18))

        color = (0, 180, 255) if self.active else (150, 150, 150)
        pygame.draw.rect(surface, color, self.rect, 2)

        text_surface = small_font.render(self.text, True, input_text_color())
        surface.blit(text_surface, (self.rect.x + 8, self.rect.y + 7))

# =========================
# BUTTON CLASS
# =========================
class Button:
    def __init__(self, x, y, w, h, text, action):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.action = action

    def draw(self, surface):
        pygame.draw.rect(surface, button_fill_color(), self.rect)
        pygame.draw.rect(surface, button_border_color(), self.rect, 1)
        text_surface = tiny_font.render(self.text, True, button_text_color())
        text_rect = text_surface.get_rect(center=self.rect.center)
        surface.blit(text_surface, text_rect)

    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)

# =========================
# COIN-D6 LiDAR PARSER
# =========================
def u16_le(lo, hi):
    return lo | (hi << 8)


def angle_from_raw(raw_angle):
    return ((raw_angle >> 1) / 64.0) % 360.0


def find_lidar_packet(buffer):
    while True:
        index = buffer.find(PACKET_HEADER)

        if index < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            return None

        if index > 0:
            del buffer[:index]

        if len(buffer) < 10:
            return None

        lsn = buffer[3]

        # Protect against false packet detection.
        if lsn == 0 or lsn > 160:
            del buffer[0]
            continue

        packet_len = 10 + lsn * 3

        if len(buffer) < packet_len:
            return None

        packet = bytes(buffer[:packet_len])
        del buffer[:packet_len]

        # FSA and LSA angle check bit should be 1.
        if not (packet[4] & 0x01) or not (packet[6] & 0x01):
            continue

        return packet


def parse_lidar_packet(packet):
    mt = packet[2]
    lsn = packet[3]

    fsa_raw = u16_le(packet[4], packet[5])
    lsa_raw = u16_le(packet[6], packet[7])

    start_angle = angle_from_raw(fsa_raw)
    end_angle = angle_from_raw(lsa_raw)

    packet_type = mt & 0x01
    new_scan = packet_type == 1

    if lsn <= 1:
        angle_step = 0.0
    else:
        end_unwrapped = end_angle
        if end_unwrapped < start_angle:
            end_unwrapped += 360.0
        angle_step = (end_unwrapped - start_angle) / (lsn - 1)

    points = []

    for i in range(lsn):
        offset = 10 + i * 3

        s_l = packet[offset]
        s_2nd = packet[offset + 1]
        s_h = packet[offset + 2]

        intensity = ((s_2nd & 0x03) * 64) + (s_l >> 2)
        distance_mm = (s_h * 64) + (s_2nd >> 2)
        angle_deg = (start_angle + angle_step * i) % 360.0

        points.append((angle_deg, distance_mm, intensity, new_scan and i == 0))

    return points


def read_lidar_data():
    """Read Coin-D6 data without blocking the Pygame screen."""
    global lidar_packet_count, last_lidar_time

    if lidar is None:
        return

    try:
        data = lidar.read(4096)
        if data:
            lidar_buffer.extend(data)

        while True:
            packet = find_lidar_packet(lidar_buffer)
            if packet is None:
                break

            lidar_packet_count += 1
            last_lidar_time = time.time()
            points = parse_lidar_packet(packet)
            now = time.time()

            for angle_deg, distance_mm, intensity, _ in points:
                if distance_mm < 50:
                    continue
                if distance_mm > max_range_m * 1000:
                    continue

                # 0.5-degree bins keep the newest point for each direction.
                angle_key = round(angle_deg * 2) / 2.0
                latest_lidar_points[angle_key] = (angle_deg, distance_mm, intensity, now)

    except Exception as e:
        serial_log.append(f"LiDAR read failed: {e}")

# =========================
# SLAM / POSE HELPERS
# =========================
def blend_angle_deg(base_deg, target_deg, alpha):
    """Blend angles without jumping at 0/360 degrees."""
    err = wrap_angle_180(target_deg - base_deg)
    return normalise_angle_360(base_deg + err * alpha)


def update_slam_pose():
    """Update the pose used for mapping and drawing.

    Raw Arduino pose remains visible in the left panel. When LiDAR scan matching
    is confident, the SLAM pose is pulled toward the LiDAR-matched pose. When the
    match is weak, it falls back to Arduino encoder/IMU pose so control remains safe.
    """
    global slam_x, slam_y, slam_heading, slam_confidence, slam_status

    if not SLAM_POSE_ENABLED:
        slam_x, slam_y, slam_heading = robot_x, robot_y, robot_heading
        slam_confidence = 0.0
        slam_status = "OFF - using Arduino pose"
        return

    if MAPPING_WAIT_FOR_FIRST_ROUTE_POINT:
        slam_x, slam_y, slam_heading = robot_x, robot_y, robot_heading
        slam_confidence = 0.0
        slam_status = "waiting for first scan point"
        return

    if LIDAR_ODOM_ENABLED and lidar_odom_confidence >= SLAM_CONFIDENCE_TO_APPLY:
        slam_x = robot_x + (lidar_odom_x - robot_x) * SLAM_BLEND_ALPHA
        slam_y = robot_y + (lidar_odom_y - robot_y) * SLAM_BLEND_ALPHA
        slam_heading = blend_angle_deg(robot_heading, lidar_odom_heading, SLAM_BLEND_ALPHA)
        slam_confidence = lidar_odom_confidence
        slam_status = f"LiDAR-corrected pose conf={slam_confidence:.2f}"
    else:
        slam_x, slam_y, slam_heading = robot_x, robot_y, robot_heading
        slam_confidence = lidar_odom_confidence
        slam_status = "Arduino pose - LiDAR match weak/building"


def current_nav_pose():
    """Pose used for mapping, drawing and route checks."""
    return slam_x, slam_y, slam_heading


def heading_to_point_deg(from_x, from_y, to_x, to_y):
    """Heading convention: 0 deg = +Y/front, +90 deg = +X/right."""
    return normalise_angle_360(math.degrees(math.atan2(to_x - from_x, to_y - from_y)))


# =========================
# COORDINATE TRANSFORMS
# =========================
def normalise_angle_360(angle_deg):
    return angle_deg % 360.0


def adjusted_lidar_angle(angle_deg):
    """Apply LiDAR-to-robot heading offset.

    Use Q/E or the on-screen LiDAR heading buttons to tune this until an object
    physically in front of the robot appears at the front of the map.
    """
    return normalise_angle_360(angle_deg + LIDAR_HEADING_OFFSET_DEG)


def lidar_to_local_xy(angle_deg, distance_mm):
    """
    COIN-D6 frame after offset correction:
    0 deg = robot front/up, 90 deg = robot right, angle increases clockwise.
    Local robot frame:
    x = right, y = forward.
    """
    theta = math.radians(adjusted_lidar_angle(angle_deg))
    local_x = distance_mm * math.sin(theta)
    local_y = distance_mm * math.cos(theta)
    return local_x, local_y


def local_to_world(local_x, local_y):
    """
    Transform LiDAR local point into fixed world map using the current SLAM pose.
    heading 0 deg means robot faces +Y. Positive heading turns toward +X.
    """
    pose_x, pose_y, pose_h = current_nav_pose()
    th = math.radians(pose_h)
    world_x = pose_x + local_x * math.cos(th) + local_y * math.sin(th)
    world_y = pose_y - local_x * math.sin(th) + local_y * math.cos(th)
    return world_x, world_y


def world_to_screen(x_mm, y_mm, center_x, center_y, scale, view_x=0.0, view_y=0.0):
    sx = center_x + int((x_mm - view_x) * scale)
    sy = center_y - int((y_mm - view_y) * scale)
    return sx, sy

# =========================
# OCCUPANCY GRID / COST MAP / A* PATH PLANNING
# =========================
def world_to_grid(x_mm, y_mm):
    return int(round(x_mm / MAP_RES_MM)), int(round(y_mm / MAP_RES_MM))


def grid_to_world(gx, gy):
    return gx * MAP_RES_MM, gy * MAP_RES_MM


def clamp_occ(value):
    return max(MAP_MIN, min(MAP_MAX, value))


def bresenham_cells(x0, y0, x1, y1):
    """Integer line cells between two grid coordinates."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0

    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


def update_occupancy_map_from_lidar():
    """Fuse recent LiDAR points into a fixed world occupancy map using SLAM pose."""
    if not MAPPING_ENABLED or MAPPING_WAIT_FOR_FIRST_ROUTE_POINT or not latest_lidar_points:
        return

    now = time.time()
    pose_x, pose_y, _pose_h = current_nav_pose()
    robot_cell = world_to_grid(pose_x, pose_y)

    for angle_deg, distance_mm, _intensity, timestamp in list(latest_lidar_points.values()):
        if now - timestamp > 0.25:
            continue
        if distance_mm < 80 or distance_mm > MAX_RAY_RANGE_MM:
            continue

        local_x, local_y = lidar_to_local_xy(angle_deg, distance_mm)
        hit_x, hit_y = local_to_world(local_x, local_y)
        hit_cell = world_to_grid(hit_x, hit_y)

        ray = bresenham_cells(robot_cell[0], robot_cell[1], hit_cell[0], hit_cell[1])
        # Mark cells along the beam as free, but keep the final hit as occupied.
        for cell in ray[:-1:2]:
            occupancy_grid[cell] = clamp_occ(occupancy_grid.get(cell, 0) - MAP_FREE_DEC)

        occupancy_grid[hit_cell] = clamp_occ(occupancy_grid.get(hit_cell, 0) + MAP_OCC_INC)


def wrap_angle_180(angle_deg):
    """Return angle in the range -180..180 degrees."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def transform_local_to_world_pose(local_x, local_y, pose_x, pose_y, pose_heading_deg):
    """Transform robot-local LiDAR point into world frame using a candidate pose."""
    th = math.radians(pose_heading_deg)
    wx = pose_x + local_x * math.cos(th) + local_y * math.sin(th)
    wy = pose_y - local_x * math.sin(th) + local_y * math.cos(th)
    return wx, wy


def get_recent_lidar_local_points(max_points=140):
    """Return recent LiDAR points already corrected into robot-local X/Y."""
    now = time.time()
    pts = []
    # Sort by distance so we keep a useful spread of wall/obstacle returns.
    data = sorted(list(latest_lidar_points.values()), key=lambda item: item[1])
    for angle_deg, distance_mm, intensity, timestamp in data:
        if now - timestamp > LIDAR_ODOM_MAX_SCAN_AGE:
            continue
        if distance_mm < 120 or distance_mm > MAX_RAY_RANGE_MM:
            continue
        lx, ly = lidar_to_local_xy(angle_deg, distance_mm)
        pts.append((lx, ly, distance_mm, intensity))
        if len(pts) >= max_points:
            break
    return pts


def occupied_cell_score(cell):
    """Soft score for how well a transformed LiDAR hit matches the fixed map."""
    gx, gy = cell
    best = 0
    # Check neighbours so small pose error still gets a partial match.
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            score = occupancy_grid.get((gx + dx, gy + dy), 0)
            if score > best:
                best = score
    if best < OCCUPIED_THRESHOLD:
        return 0.0
    return min(1.0, best / float(MAP_MAX))


def score_lidar_pose(local_points, pose_x, pose_y, pose_heading_deg):
    """Score a candidate pose by matching live LiDAR hit points to the fixed map."""
    if not local_points:
        return 0.0
    total = 0.0
    used = 0
    for lx, ly, _dist, intensity in local_points[::2]:
        wx, wy = transform_local_to_world_pose(lx, ly, pose_x, pose_y, pose_heading_deg)
        cell = world_to_grid(wx, wy)
        match = occupied_cell_score(cell)
        if match > 0.0:
            # Stronger LiDAR returns slightly help, but do not dominate the match.
            intensity_weight = 1.0 + min(0.30, max(0, intensity) / 255.0 * 0.30)
            total += match * intensity_weight
        used += 1
    return total / max(1, used)


def update_lidar_odometry_verification():
    """Estimate LiDAR-based pose correction and compare it with Arduino odometry.

    The Arduino still provides the live X/Y/yaw used for control. This function is
    a verification layer: it performs a small scan-matching search around the
    encoder/IMU pose and reports the LiDAR-matched pose and error.
    """
    global last_lidar_odom_time, lidar_odom_x, lidar_odom_y, lidar_odom_heading
    global lidar_odom_error_x, lidar_odom_error_y, lidar_odom_error_heading, lidar_odom_error_dist
    global lidar_odom_confidence, lidar_odom_match_score, lidar_odom_points_used, lidar_odom_status

    if not LIDAR_ODOM_ENABLED:
        lidar_odom_status = "OFF"
        return

    now = time.time()
    if now - last_lidar_odom_time < LIDAR_ODOM_INTERVAL:
        return
    last_lidar_odom_time = now

    map_cells = sum(1 for score in occupancy_grid.values() if score >= OCCUPIED_THRESHOLD)
    if map_cells < LIDAR_ODOM_MIN_MAP_CELLS:
        lidar_odom_status = f"building fixed map ({map_cells}/{LIDAR_ODOM_MIN_MAP_CELLS})"
        lidar_odom_confidence = 0.0
        return

    local_points = get_recent_lidar_local_points()
    lidar_odom_points_used = len(local_points)
    if len(local_points) < LIDAR_ODOM_MIN_POINTS:
        lidar_odom_status = f"not enough scan points ({len(local_points)}/{LIDAR_ODOM_MIN_POINTS})"
        lidar_odom_confidence = 0.0
        return

    best = None
    xy_steps = []
    v = -LIDAR_ODOM_SEARCH_XY_MM
    while v <= LIDAR_ODOM_SEARCH_XY_MM + 0.001:
        xy_steps.append(v)
        v += LIDAR_ODOM_SEARCH_XY_STEP_MM

    yaw_steps = []
    yv = -LIDAR_ODOM_SEARCH_YAW_DEG
    while yv <= LIDAR_ODOM_SEARCH_YAW_DEG + 0.001:
        yaw_steps.append(yv)
        yv += LIDAR_ODOM_SEARCH_YAW_STEP_DEG

    for dx in xy_steps:
        for dy in xy_steps:
            # Avoid unrealistic jumps for the verification search.
            if math.hypot(dx, dy) > LIDAR_ODOM_MAX_ACCEPT_XY_MM:
                continue
            for dh in yaw_steps:
                cand_x = robot_x + dx
                cand_y = robot_y + dy
                cand_h = robot_heading + dh
                score = score_lidar_pose(local_points, cand_x, cand_y, cand_h)
                # Penalise large corrections so a similar score keeps the Arduino pose.
                penalty = 0.00035 * math.hypot(dx, dy) + 0.006 * abs(dh)
                ranked = score - penalty
                if best is None or ranked > best[0]:
                    best = (ranked, score, cand_x, cand_y, cand_h, dx, dy, dh)

    if best is None:
        lidar_odom_status = "no candidate pose"
        lidar_odom_confidence = 0.0
        return

    _ranked, raw_score, bx, by, bh, dx, dy, dh = best
    lidar_odom_x = bx
    lidar_odom_y = by
    lidar_odom_heading = normalise_angle_360(bh)
    lidar_odom_error_x = dx
    lidar_odom_error_y = dy
    lidar_odom_error_heading = wrap_angle_180(dh)
    lidar_odom_error_dist = math.hypot(dx, dy)
    lidar_odom_match_score = raw_score
    lidar_odom_confidence = max(0.0, min(1.0, raw_score / max(0.001, LIDAR_ODOM_GOOD_SCORE)))

    if raw_score < LIDAR_ODOM_GOOD_SCORE * 0.55:
        lidar_odom_status = f"weak match score={raw_score:.2f}"
    elif lidar_odom_error_dist > 120 or abs(lidar_odom_error_heading) > 6:
        lidar_odom_status = f"CHECK drift: {lidar_odom_error_dist:.0f}mm, {lidar_odom_error_heading:+.1f}deg"
    else:
        lidar_odom_status = f"OK: {lidar_odom_error_dist:.0f}mm, {lidar_odom_error_heading:+.1f}deg"


def toggle_lidar_odometry():
    global LIDAR_ODOM_ENABLED, lidar_odom_status, last_message
    LIDAR_ODOM_ENABLED = not LIDAR_ODOM_ENABLED
    lidar_odom_status = "ON" if LIDAR_ODOM_ENABLED else "OFF"
    last_message = f"LiDAR odometry verification {lidar_odom_status}"


def start_mapping_after_first_route_point(reason="planned route"):
    """Pause map updates until the robot reaches the first route coordinate."""
    global MAPPING_ENABLED, MAPPING_WAIT_FOR_FIRST_ROUTE_POINT, MAPPING_FIRST_ROUTE_POINT, mapping_status

    if not path_waypoints:
        MAPPING_ENABLED = True
        MAPPING_WAIT_FOR_FIRST_ROUTE_POINT = False
        MAPPING_FIRST_ROUTE_POINT = None
        mapping_status = "mapping live - no first route gate"
        return

    # Use a short first scan-start point along the planned path. This avoids waiting
    # until the final goal when the path is a simple straight line with only one
    # Arduino waypoint.
    if planned_path and len(planned_path) >= 2:
        total = path_length(planned_path)
        restart_dist = min(total, max(180.0, min(350.0, total * 0.25)))
        first_scan_pt = interpolate_path_by_distance(planned_path, restart_dist)
        MAPPING_FIRST_ROUTE_POINT = first_scan_pt if first_scan_pt is not None else path_waypoints[0]
    else:
        MAPPING_FIRST_ROUTE_POINT = path_waypoints[0]

    MAPPING_WAIT_FOR_FIRST_ROUTE_POINT = True
    MAPPING_ENABLED = False
    mapping_status = (
        f"mapping paused until first scan point ({int(MAPPING_FIRST_ROUTE_POINT[0])},"
        f"{int(MAPPING_FIRST_ROUTE_POINT[1])}) before restarting map"
    )


def update_mapping_start_gate():
    """Restart scanning/map update after first rotate + first coordinate reach.

    The program cannot directly read an Arduino 'rotation finished' flag, so it uses
    the live pose: once the robot is close to the first route point, it clears the
    temporary map and starts a fresh fixed map from that known coordinate.
    """
    global MAPPING_ENABLED, MAPPING_WAIT_FOR_FIRST_ROUTE_POINT, MAPPING_FIRST_ROUTE_POINT, mapping_status
    global planning_message, last_message

    if not MAPPING_WAIT_FOR_FIRST_ROUTE_POINT or MAPPING_FIRST_ROUTE_POINT is None:
        return

    pose_x, pose_y, pose_h = current_nav_pose()
    first_x, first_y = MAPPING_FIRST_ROUTE_POINT
    dist = math.hypot(first_x - pose_x, first_y - pose_y)

    if dist > MAPPING_FIRST_POINT_TOLERANCE_MM:
        mapping_status = f"mapping paused until first scan point reached | dist={dist:.0f}mm"
        return

    if MAPPING_RESTART_CLEAR_MAP:
        occupancy_grid.clear()
        cost_grid.clear()
        latest_lidar_points.clear()

    MAPPING_WAIT_FOR_FIRST_ROUTE_POINT = False
    MAPPING_FIRST_ROUTE_POINT = None
    MAPPING_ENABLED = True
    mapping_status = "mapping restarted at first route point"
    planning_message = "SLAM map restarted after first route point"
    last_message = "First route point reached - SLAM scanning restarted"


def clear_fixed_map():
    global planning_message
    global MAPPING_ENABLED, MAPPING_WAIT_FOR_FIRST_ROUTE_POINT, MAPPING_FIRST_ROUTE_POINT, mapping_status
    occupancy_grid.clear()
    cost_grid.clear()
    planned_path.clear()
    path_waypoints.clear()
    global lidar_odom_status, lidar_odom_confidence, slam_confidence, slam_status
    lidar_odom_status = "waiting for map"
    lidar_odom_confidence = 0.0
    slam_confidence = 0.0
    slam_status = "Arduino pose only"
    MAPPING_ENABLED = True
    MAPPING_WAIT_FOR_FIRST_ROUTE_POINT = False
    MAPPING_FIRST_ROUTE_POINT = None
    mapping_status = "mapping live"
    planning_message = "Fixed map cleared"


def build_cost_map():
    """Create a collision cost map around occupied cells using the 200 mm robot frame radius."""
    cost_grid.clear()
    occupied = [cell for cell, score in occupancy_grid.items() if score >= OCCUPIED_THRESHOLD]

    for ox, oy in occupied:
        cost_grid[(ox, oy)] = COST_OBSTACLE
        for dx in range(-INFLATION_CELLS, INFLATION_CELLS + 1):
            for dy in range(-INFLATION_CELLS, INFLATION_CELLS + 1):
                d = math.hypot(dx, dy)
                if d > INFLATION_CELLS:
                    continue
                cell = (ox + dx, oy + dy)
                if cell == (ox, oy):
                    continue
                # Cost is higher near obstacles and lower at the edge of inflation.
                # Within the robot body radius, treat the cell as fully blocked.
                if d <= ROBOT_RADIUS_CELLS:
                    inflated = COST_OBSTACLE
                else:
                    inflated = int(220 * (1.0 - d / (INFLATION_CELLS + 0.001)))
                cost_grid[cell] = max(cost_grid.get(cell, 0), inflated)


def is_cell_blocked(cell):
    return cost_grid.get(cell, 0) >= 240


def astar_plan(start_xy, goal_xy):
    """Plan a path on the current cost map using A*. Returns world-mm points."""
    build_cost_map()

    start = world_to_grid(*start_xy)
    goal = world_to_grid(*goal_xy)

    if is_cell_blocked(start):
        return None, "Start is inside obstacle/cost area"
    if is_cell_blocked(goal):
        return None, "Goal is inside obstacle/cost area"

    min_x = min([start[0], goal[0]] + [c[0] for c in occupancy_grid.keys()] + [0]) - 30
    max_x = max([start[0], goal[0]] + [c[0] for c in occupancy_grid.keys()] + [0]) + 30
    min_y = min([start[1], goal[1]] + [c[1] for c in occupancy_grid.keys()] + [0]) - 30
    max_y = max([start[1], goal[1]] + [c[1] for c in occupancy_grid.keys()] + [0]) + 30

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    neighbors = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
    ]

    open_heap = []
    heapq.heappush(open_heap, (0.0, start))
    came_from = {}
    g_score = {start: 0.0}
    visited = set()
    max_iterations = 60000
    iterations = 0

    while open_heap and iterations < max_iterations:
        iterations += 1
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            cells = [current]
            while current in came_from:
                current = came_from[current]
                cells.append(current)
            cells.reverse()
            return [grid_to_world(x, y) for x, y in cells], f"Path found: {len(cells)} cells"

        for dx, dy, base_cost in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt[0] < min_x or nxt[0] > max_x or nxt[1] < min_y or nxt[1] > max_y:
                continue
            if is_cell_blocked(nxt):
                continue
            extra = cost_grid.get(nxt, 0) / 80.0
            tentative = g_score[current] + base_cost + extra
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                f = tentative + heuristic(nxt, goal)
                heapq.heappush(open_heap, (f, nxt))

    return None, "No path found"


def smooth_path(points):
    """Simple line-of-sight path simplification."""
    if not points or len(points) <= 2:
        return points or []

    def clear_line(a, b):
        ac = world_to_grid(*a)
        bc = world_to_grid(*b)
        for cell in bresenham_cells(ac[0], ac[1], bc[0], bc[1]):
            if is_cell_blocked(cell):
                return False
        return True

    result = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1:
            if clear_line(points[i], points[j]):
                break
            j -= 1
        result.append(points[j])
        i = j
    return result


def decimate_path(points, spacing_mm=250):
    """Reduce dense path points using a minimum spacing, while keeping the final goal."""
    if not points:
        return []
    output = [points[0]]
    last = points[0]
    for p in points[1:]:
        if math.hypot(p[0] - last[0], p[1] - last[1]) >= spacing_mm:
            output.append(p)
            last = p
    if output[-1] != points[-1]:
        output.append(points[-1])
    return output


def path_length(points):
    """Total path length in mm."""
    if not points or len(points) < 2:
        return 0.0
    return sum(math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]) for i in range(len(points) - 1))


def interpolate_path_by_distance(points, distance_mm):
    """Return a point located distance_mm along a polyline."""
    if not points:
        return None
    if distance_mm <= 0:
        return points[0]

    walked = 0.0
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        seg = math.hypot(x2 - x1, y2 - y1)
        if seg <= 1e-6:
            continue
        if walked + seg >= distance_mm:
            r = (distance_mm - walked) / seg
            return (x1 + (x2 - x1) * r, y1 + (y2 - y1) * r)
        walked += seg
    return points[-1]


def make_arduino_route_points(points, max_points=MAX_ROUTE_POINTS):
    """Create 1..10 ordered route points from the planned path.

    The full A* / smoothed path can contain many cells. The Arduino command queue is
    limited, so this function converts the route into a small ordered list of useful
    waypoints:
    - point 1 is the first forward command after the current robot position
    - the final point is always the target goal
    - if there are many points, it samples them evenly along distance
    - if there are obstacle-avoidance turns, the smoothed path corners are preserved
      as much as possible before reducing to 10 points
    """
    if not points or len(points) < 2:
        return []

    # Remove the current robot/start point from the command list.
    command_path = points[1:]
    if len(command_path) <= max_points:
        return command_path

    # Prefer a spaced route first so points are not too close together.
    spaced = decimate_path(points, spacing_mm=ROUTE_POINT_SPACING_MM)[1:]
    if 1 <= len(spaced) <= max_points:
        return spaced

    # Guaranteed 1..10 route: sample by travelled distance and always include goal.
    total = path_length(points)
    if total <= 1e-6:
        return [points[-1]]

    route = []
    for i in range(1, max_points + 1):
        d = total * i / max_points
        pt = interpolate_path_by_distance(points, d)
        if pt is None:
            continue
        if not route or math.hypot(pt[0] - route[-1][0], pt[1] - route[-1][1]) >= 1.0:
            route.append(pt)

    # Make sure the last route point is exactly the requested goal.
    if route:
        route[-1] = points[-1]
    else:
        route = [points[-1]]

    return route[:max_points]



def plan_path_to_goal(goal, reason="manual plan"):
    """Plan a safe route from the live robot pose to a selected goal.

    This function is used by the Plan Route button.
    It calculates a safe route around the currently mapped obstacles.
    """
    global planned_path, path_waypoints, planning_message, last_message

    nav_x, nav_y, _ = current_nav_pose()
    start = (nav_x, nav_y)
    path, msg = astar_plan(start, goal)
    if path is None:
        planned_path = []
        path_waypoints = []
        planning_message = f"{reason}: {msg}"
        last_message = planning_message
        return False

    smoothed = smooth_path(path)
    if len(smoothed) < 2:
        smoothed = path

    planned_path = smoothed
    path_waypoints = make_arduino_route_points(smoothed, max_points=MAX_ROUTE_POINTS)

    if not path_waypoints:
        planning_message = "Path found, but no valid route point was created"
        last_message = planning_message
        return False

    planning_message = (
        f"{reason}: {msg} | route points: {len(path_waypoints)}/10 | "
        f"robot radius {ROBOT_RADIUS_MM}mm"
    )
    last_message = f"Route ready: {len(path_waypoints)} point(s) | {reason}"
    return True


def plan_path_to_input_target():
    """Plan a safe route for the robot centre, keeping the 200 mm robot frame clear of obstacles."""
    goal = (x_box.value_int(0), y_box.value_int(0))
    plan_path_to_goal(goal, reason="manual plan")


def send_path_as_waypoints():
    """Send the planned path to Arduino as a normal waypoint queue."""
    global last_message
    if not path_waypoints:
        last_message = "No path to send. Press Plan first."
        return
    if arduino is None:
        last_message = "Arduino not connected"
        return

    # path_waypoints is already a 1..10 ordered obstacle-avoiding route.
    pts = path_waypoints[:MAX_ROUTE_POINTS]

    cmd = " ".join(f"({int(x)},{int(y)},0)" for x, y in pts) + ":\n"
    try:
        arduino.write(cmd.encode("utf-8"))
        arduino.flush()
        start_mapping_after_first_route_point("send planned path")
        last_message = f"Sent planned path: {len(pts)} waypoints | mapping waits for R1"
    except Exception as e:
        last_message = f"Path send failed: {e}"



# =========================
# ARDUINO SERIAL FUNCTIONS
# =========================
def build_command():
    if not waypoints:
        return None
    return " ".join(f"({x},{y},{wait_sec})" for x, y, wait_sec in waypoints) + ":\n"


def send_waypoints():
    global last_message
    command = build_command()
    if command is None:
        last_message = "No waypoint in queue"
        return

    if arduino is None:
        last_message = "Arduino not connected"
        print(last_message)
        return

    try:
        arduino.write(command.encode("utf-8"))
        arduino.flush()
        last_message = f"Sent: {command.strip()}"
        print(last_message)
    except Exception as e:
        last_message = f"Serial send failed: {e}"
        print(last_message)


def read_arduino_lines():
    """Read Arduino feedback without blocking the Pygame screen."""
    global last_message, robot_x, robot_y, robot_heading, last_arduino_time, arduino_rx_count, last_arduino_raw

    if arduino is None:
        return

    try:
        data = arduino.read(4096)
        if not data:
            return

        arduino_line_buffer.extend(data)

        while b"\n" in arduino_line_buffer:
            line_bytes, _, remaining = arduino_line_buffer.partition(b"\n")
            arduino_line_buffer[:] = remaining
            raw = line_bytes.decode("utf-8", errors="ignore").strip()

            if not raw:
                continue

            arduino_rx_count += 1
            last_arduino_raw = raw
            serial_log.append(raw)

            # Arduino pose line format: x>y>heading, e.g. 120.0>400.5>-1.25
            match = re.match(r"^\s*(-?\d+(?:\.\d+)?)>(-?\d+(?:\.\d+)?)>(-?\d+(?:\.\d+)?)\s*$", raw)
            if match:
                robot_x = float(match.group(1))
                robot_y = float(match.group(2))
                robot_heading = float(match.group(3))
                last_arduino_time = time.time()
                last_message = f"Pose: X={robot_x:.1f} mm, Y={robot_y:.1f} mm, H={robot_heading:.2f} deg"
            else:
                last_message = raw

    except Exception as e:
        last_message = f"Arduino read failed: {e}"

# =========================
# OBSTACLE SAFETY STOP
# =========================
def send_stop_command(reason="Manual stop"):
    """Send emergency stop to Arduino. Arduino must support STOP: command."""
    global last_message, last_stop_command_time

    last_stop_command_time = time.time()

    if arduino is None:
        last_message = "STOP needed, but Arduino not connected"
        serial_log.append(last_message)
        return

    try:
        arduino.write(b"STOP:\n")
        arduino.flush()
        last_message = f"STOP sent: {reason}"
        serial_log.append(last_message)
        print(last_message)
    except Exception as e:
        last_message = f"STOP send failed: {e}"
        serial_log.append(last_message)


def front_angle_error(angle_deg):
    """Return signed angle difference from robot front, where 0 deg is forward."""
    corrected = adjusted_lidar_angle(angle_deg)
    return ((corrected + 180.0) % 360.0) - 180.0


def update_front_obstacle_stop():
    """Check LiDAR points in front of the robot body and stop if clearance is too small."""
    global obstacle_detected, front_obstacle_distance_mm, front_obstacle_clearance_mm

    if not AUTO_STOP_ENABLED:
        obstacle_detected = False
        front_obstacle_distance_mm = None
        front_obstacle_clearance_mm = None
        return

    now = time.time()
    nearest = None
    nearest_clearance = None
    lateral_limit = ROBOT_RADIUS_MM + FRONT_STOP_LATERAL_MARGIN_MM

    for angle_deg, distance_mm, _intensity, timestamp in list(latest_lidar_points.values()):
        if now - timestamp > 0.30:
            continue
        if distance_mm <= 0 or distance_mm > FRONT_STOP_DISTANCE_MM + 50:
            continue

        local_x, local_y = lidar_to_local_xy(angle_deg, distance_mm)

        # Only consider points physically in front of the robot, not behind or beside it.
        if local_y <= 0:
            continue
        if abs(local_x) > lateral_limit:
            continue
        if abs(front_angle_error(angle_deg)) > max(FRONT_CONE_HALF_ANGLE_DEG, 28):
            continue

        clearance = local_y - ROBOT_RADIUS_MM
        if nearest_clearance is None or clearance < nearest_clearance:
            nearest_clearance = clearance
            nearest = distance_mm

    front_obstacle_distance_mm = nearest
    front_obstacle_clearance_mm = nearest_clearance
    obstacle_detected = nearest_clearance is not None and nearest_clearance < FRONT_STOP_CLEARANCE_MM

    if obstacle_detected and (now - last_stop_command_time) >= STOP_REPEAT_SECONDS:
        send_stop_command(f"front clearance {nearest_clearance:.0f} mm")

# =========================
# UI ACTIONS
# =========================
x_box = InputBox(25, 70, 80, 35, "X mm", "0")
y_box = InputBox(120, 70, 80, 35, "Y mm", "1000")
wait_box = InputBox(215, 70, 100, 35, "Wait sec", "0")
input_boxes = [x_box, y_box, wait_box]


def add_waypoint():
    global last_message
    x = x_box.value_int(0)
    y = y_box.value_int(0)
    wait_sec = max(0, wait_box.value_int(0))

    if len(waypoints) >= 10:
        last_message = "Arduino supports max 10 waypoints"
        return

    waypoints.append((x, y, wait_sec))
    last_message = f"Added waypoint: ({x},{y},{wait_sec})"


def send_single_now():
    global last_message
    x = x_box.value_int(0)
    y = y_box.value_int(0)
    wait_sec = max(0, wait_box.value_int(0))
    temp = f"({x},{y},{wait_sec}):\n"

    if arduino is None:
        last_message = "Arduino not connected"
        return

    try:
        arduino.write(temp.encode("utf-8"))
        arduino.flush()
        last_message = f"Sent single: {temp.strip()}"
    except Exception as e:
        last_message = f"Serial send failed: {e}"


def clear_waypoints():
    global last_message
    waypoints.clear()
    last_message = "Waypoint queue cleared"


def clear_lidar_map():
    global last_message
    latest_lidar_points.clear()
    last_message = "LiDAR map cleared"


def manual_stop():
    send_stop_command("manual button")


def clear_fixed_map_for_lidar_recalibration():
    """Clear map data that was created using the previous LiDAR heading offset."""
    global planning_message
    occupancy_grid.clear()
    cost_grid.clear()
    planned_path.clear()
    path_waypoints.clear()
    global lidar_odom_status, lidar_odom_confidence
    lidar_odom_status = "waiting for map"
    lidar_odom_confidence = 0.0
    planning_message = "Fixed map cleared after LiDAR heading change"


def adjust_lidar_heading(delta_deg):
    """Rotate LiDAR points relative to robot heading without changing Arduino pose."""
    global LIDAR_HEADING_OFFSET_DEG, last_message
    LIDAR_HEADING_OFFSET_DEG = ((LIDAR_HEADING_OFFSET_DEG + delta_deg + 180.0) % 360.0) - 180.0
    clear_fixed_map_for_lidar_recalibration()
    last_message = f"LiDAR heading offset: {LIDAR_HEADING_OFFSET_DEG:+.1f} deg | fixed map cleared"


def reset_lidar_heading():
    global LIDAR_HEADING_OFFSET_DEG, last_message
    LIDAR_HEADING_OFFSET_DEG = 0.0
    clear_fixed_map_for_lidar_recalibration()
    last_message = "LiDAR heading offset reset to 0.0 deg | fixed map cleared"


def clear_local_robot_state_after_arduino_reset():
    """Clear local Python state that depends on Arduino odometry."""
    global robot_x, robot_y, robot_heading, last_arduino_time
    global planning_message

    robot_x = 0.0
    robot_y = 0.0
    robot_heading = 0.0
    last_arduino_time = 0.0
    global lidar_odom_x, lidar_odom_y, lidar_odom_heading, lidar_odom_error_x, lidar_odom_error_y
    global lidar_odom_error_heading, lidar_odom_error_dist, lidar_odom_confidence, lidar_odom_status
    global slam_x, slam_y, slam_heading, slam_confidence, slam_status
    global MAPPING_ENABLED, MAPPING_WAIT_FOR_FIRST_ROUTE_POINT, MAPPING_FIRST_ROUTE_POINT, mapping_status
    lidar_odom_x = 0.0
    lidar_odom_y = 0.0
    lidar_odom_heading = 0.0
    lidar_odom_error_x = 0.0
    lidar_odom_error_y = 0.0
    lidar_odom_error_heading = 0.0
    lidar_odom_error_dist = 0.0
    lidar_odom_confidence = 0.0
    lidar_odom_status = "waiting for map"
    slam_x = 0.0
    slam_y = 0.0
    slam_heading = 0.0
    slam_confidence = 0.0
    slam_status = "Arduino pose only"
    MAPPING_ENABLED = True
    MAPPING_WAIT_FOR_FIRST_ROUTE_POINT = False
    MAPPING_FIRST_ROUTE_POINT = None
    mapping_status = "mapping live"

    arduino_line_buffer.clear()
    serial_log.clear()
    latest_lidar_points.clear()
    occupancy_grid.clear()
    cost_grid.clear()
    planned_path.clear()
    path_waypoints.clear()
    waypoints.clear()

    planning_message = "Arduino reset - map and route cleared"


def reset_arduino_uno():
    """Reset all: reset Arduino Uno using USB serial DTR, then clear local map/route state."""
    global arduino, last_message

    clear_local_robot_state_after_arduino_reset()

    if arduino is None:
        last_message = "Arduino reset failed: Arduino not connected"
        serial_log.append(last_message)
        return

    try:
        # On Arduino Uno, toggling DTR over USB triggers the auto-reset circuit.
        try:
            arduino.write(b"STOP:\n")
            arduino.flush()
        except Exception:
            pass

        arduino.setDTR(False)
        time.sleep(0.25)
        arduino.setDTR(True)

        # Uno bootloader needs a short moment before it starts printing again.
        time.sleep(2.0)
        arduino.reset_input_buffer()
        arduino.reset_output_buffer()

        last_message = "Arduino Uno reset by USB DTR | map, route and pose cleared"
        serial_log.append(last_message)
        print(last_message)

    except Exception as e:
        last_message = f"Arduino reset failed: {e}"
        serial_log.append(last_message)


buttons = [
    Button(25, 125, 90, 32, "Add", add_waypoint),
    Button(125, 125, 90, 32, "Send One", send_single_now),
    Button(225, 125, 90, 32, "Send All", send_waypoints),
    Button(25, 165, 90, 30, "STOP", manual_stop),
    Button(125, 165, 90, 30, "Clear WP", clear_waypoints),
    Button(225, 165, 90, 30, "Clear Scan", clear_lidar_map),
    Button(25, 202, 90, 30, "Plan Route", plan_path_to_input_target),
    Button(125, 202, 190, 30, "Send Path", send_path_as_waypoints),
    Button(25, 239, 290, 28, "Clear Fixed Map", clear_fixed_map),
    Button(25, 274, 90, 28, "LiDAR -5°", lambda: adjust_lidar_heading(-LIDAR_OFFSET_STEP_DEG)),
    Button(125, 274, 90, 28, "LiDAR +5°", lambda: adjust_lidar_heading(LIDAR_OFFSET_STEP_DEG)),
    Button(225, 274, 90, 28, "Reset LiDAR", reset_lidar_heading),
    Button(25, 309, 140, 28, "LiDAR Odom ON/OFF", toggle_lidar_odometry),
    Button(175, 309, 140, 28, "Reset All", reset_arduino_uno),
    Button(25, 344, 290, 28, "White / Dark Background", toggle_background_theme),
]

# =========================
# DRAWING FUNCTIONS
# =========================
LEFT_PANEL_W = 330
MARGIN = 25
lidar_rect = pygame.Rect(350, 45, 520, 370)
status_rect = pygame.Rect(25, 435, 845, 45)
serial_log_pos = (580, 424)


def update_layout():
    """Resize UI panels based on the current window size."""
    global lidar_rect, status_rect, serial_log_pos

    # Left controls stay readable; the LiDAR map takes the remaining space.
    lidar_x = LEFT_PANEL_W + 20
    lidar_y = 45
    lidar_w = max(300, WIDTH - lidar_x - MARGIN)
    lidar_h = max(230, HEIGHT - 130)
    lidar_rect = pygame.Rect(lidar_x, lidar_y, lidar_w, lidar_h)

    status_rect = pygame.Rect(MARGIN, HEIGHT - 78, WIDTH - 2 * MARGIN, 58)

    # Keep Arduino feedback on the right side of the status area when there is space.
    serial_x = max(lidar_rect.x + 230, WIDTH - 320)
    serial_y = max(lidar_rect.bottom + 6, HEIGHT - 76)
    serial_log_pos = (serial_x, serial_y)


update_layout()


def draw_text_fit(surface, text, font_obj, color, pos, max_width_px):
    """Draw one line of text and shorten it if it would overflow the panel."""
    if max_width_px <= 30:
        return
    shown = str(text)
    if font_obj.size(shown)[0] > max_width_px:
        while len(shown) > 4 and font_obj.size(shown + "...")[0] > max_width_px:
            shown = shown[:-1]
        shown = shown + "..."
    surface.blit(font_obj.render(shown, True, color), pos)


def draw_waypoint_list(surface):
    title = small_font.render("Waypoint Queue / Planned Route", True, main_text_color())
    surface.blit(title, (25, 418))

    path_status = tiny_font.render(planning_message[:42], True, (180, 220, 180))
    surface.blit(path_status, (25, 442))

    route_count = tiny_font.render(f"Route points: {len(path_waypoints)}/10", True, (180, 180, 220))
    surface.blit(route_count, (25, 460))

    y0 = 482
    if path_waypoints:
        route_title = tiny_font.render("Planned route points to avoid obstacles:", True, (230, 230, 120))
        surface.blit(route_title, (25, y0))
        # Show all route points, but keep the panel compact.
        for i, (x, y) in enumerate(path_waypoints[:MAX_ROUTE_POINTS], start=1):
            line = tiny_font.render(f"R{i}. ({int(x)}, {int(y)})", True, secondary_text_color())
            col = 0 if i <= 5 else 1
            row = (i - 1) % 5
            surface.blit(line, (25 + col * 145, y0 + 18 + row * 17))
        y0 += 18 + min(5, len(path_waypoints)) * 17 + 8

    manual_title = tiny_font.render("Manual queue:", True, muted_text_color())
    surface.blit(manual_title, (25, y0))

    if not waypoints:
        empty = tiny_font.render("No manual waypoints added", True, muted_text_color())
        surface.blit(empty, (25, y0 + 18))
        return

    start_index = max(0, len(waypoints) - 3)
    for row, (x, y, w) in enumerate(waypoints[start_index:], start=start_index + 1):
        line = tiny_font.render(f"{row}. ({x}, {y}, {w})", True, secondary_text_color())
        surface.blit(line, (25, y0 + 18 + (row - start_index - 1) * 18))


def draw_lidar_screen(surface, fps):
    pygame.draw.rect(surface, map_panel_color(), lidar_rect)
    pygame.draw.rect(surface, highlight_text_color(), lidar_rect, 2)

    title = small_font.render("COIN-D6 LiDAR + Arduino Odometry Map", True, highlight_text_color())
    surface.blit(title, (lidar_rect.x + 12, lidar_rect.y + 10))
    offset_label = tiny_font.render(f"LiDAR heading offset: {LIDAR_HEADING_OFFSET_DEG:+.1f} deg", True, serial_warning_color())
    surface.blit(offset_label, (max(lidar_rect.x + 260, lidar_rect.right - 240), lidar_rect.y + 13))

    # Reserve clean space for title at the top and status text at the bottom.
    # The actual map/robot marker is centred inside the remaining green box area.
    top_pad = 42
    bottom_pad = 116
    plot_area = pygame.Rect(
        lidar_rect.x + 12,
        lidar_rect.y + top_pad,
        lidar_rect.width - 24,
        max(120, lidar_rect.height - top_pad - bottom_pad),
    )
    center_x = plot_area.centerx
    center_y = plot_area.centery
    plot_radius = max(45, min(plot_area.width, plot_area.height) // 2 - 8)
    scale = plot_radius / (max_range_m * 1000.0)  # pixels per mm

    # Fixed world view: the room/map stays still while the robot marker moves inside it.
    if MAP_VIEW_FIXED:
        view_x = MAP_VIEW_CENTER_X
        view_y = MAP_VIEW_CENTER_Y
    else:
        view_x, view_y, _ = current_nav_pose()

    nav_x, nav_y, nav_h = current_nav_pose()
    robot_sx, robot_sy = world_to_screen(nav_x, nav_y, center_x, center_y, scale, view_x, view_y)

    pygame.draw.rect(surface, map_plot_border_color(), plot_area, 1)

    # LiDAR range rings centred on the moving robot marker.
    max_whole_m = int(max_range_m)
    if plot_area.collidepoint(robot_sx, robot_sy):
        for r_m in range(1, max_whole_m + 1):
            radius = int(r_m * 1000 * scale)
            pygame.draw.circle(surface, map_grid_color(), (robot_sx, robot_sy), radius, 1)
            label = tiny_font.render(f"{r_m}m", True, muted_text_color())
            surface.blit(label, (min(plot_area.right - 30, robot_sx + radius + 4), robot_sy - 8))

    # Fixed world axes through the map origin.
    origin_sx, origin_sy = world_to_screen(0.0, 0.0, center_x, center_y, scale, view_x, view_y)
    if plot_area.left <= origin_sx <= plot_area.right:
        pygame.draw.line(surface, map_axis_color(), (origin_sx, plot_area.top), (origin_sx, plot_area.bottom), 1)
    if plot_area.top <= origin_sy <= plot_area.bottom:
        pygame.draw.line(surface, map_axis_color(), (plot_area.left, origin_sy), (plot_area.right, origin_sy), 1)
    if plot_area.collidepoint(origin_sx, origin_sy):
        pygame.draw.circle(surface, muted_text_color(), (origin_sx, origin_sy), 4, 1)

    # Front safety stop zone. Anything inside this body-clearance zone triggers STOP.
    stop_radius = int(FRONT_STOP_DISTANCE_MM * scale)
    left_angle = math.radians(nav_h - FRONT_CONE_HALF_ANGLE_DEG)
    right_angle = math.radians(nav_h + FRONT_CONE_HALF_ANGLE_DEG)
    left_pt = (robot_sx + int(math.sin(left_angle) * stop_radius), robot_sy - int(math.cos(left_angle) * stop_radius))
    right_pt = (robot_sx + int(math.sin(right_angle) * stop_radius), robot_sy - int(math.cos(right_angle) * stop_radius))
    zone_color = (220, 40, 40) if obstacle_detected else ((150, 80, 80) if WHITE_BACKGROUND else (100, 70, 70))
    if plot_area.collidepoint(robot_sx, robot_sy):
        pygame.draw.line(surface, zone_color, (robot_sx, robot_sy), left_pt, 1)
        pygame.draw.line(surface, zone_color, (robot_sx, robot_sy), right_pt, 1)
        # Draw a simple front trigger rectangle as well. This matches the body-clearance check.
        front_x = robot_sx + int(math.sin(math.radians(nav_h)) * stop_radius)
        front_y = robot_sy - int(math.cos(math.radians(nav_h)) * stop_radius)
        pygame.draw.line(surface, zone_color, (robot_sx, robot_sy), (front_x, front_y), 2)
        stop_label = f"STOP {FRONT_STOP_CLEARANCE_MM}mm clearance"
        draw_text_fit(surface, stop_label, tiny_font, zone_color, (front_x + 6, front_y - 8), 150)

    # Draw the real robot frame clearance.
    # This circle represents the 200 mm radius from the robot centre.
    robot_radius_px = max(4, int(ROBOT_RADIUS_MM * scale))
    if plot_area.collidepoint(robot_sx, robot_sy):
        pygame.draw.circle(surface, robot_frame_color(), (robot_sx, robot_sy), robot_radius_px, 1)

    # Keep the robot-frame text outside the map centre so it does not cover LiDAR points.
    # Important: the emergency stop is based on CLEARANCE from the robot frame, not
    # from the robot centre. With a 200 mm robot radius and 250 mm clearance,
    # the LiDAR trigger distance from the centre is 450 mm.
    frame_box = pygame.Rect(plot_area.right - 230, plot_area.y + 8, 220, 66)
    pygame.draw.rect(surface, map_frame_box_color(), frame_box)
    pygame.draw.rect(surface, map_frame_box_border_color(), frame_box, 1)
    draw_text_fit(surface, f"Robot frame: R={ROBOT_RADIUS_MM} mm", tiny_font, main_text_color(), (frame_box.x + 8, frame_box.y + 6), frame_box.width - 16)
    draw_text_fit(surface, f"Cost clearance: {INFLATION_RADIUS_MM} mm", tiny_font, highlight_text_color(), (frame_box.x + 8, frame_box.y + 23), frame_box.width - 16)
    draw_text_fit(surface, f"Front stop: {FRONT_STOP_CLEARANCE_MM} mm clearance", tiny_font, serial_warning_color(), (frame_box.x + 8, frame_box.y + 40), frame_box.width - 16)

    # Draw fixed occupancy map and inflated cost map
    map_drawn = 0
    for (gx, gy), score in list(occupancy_grid.items()):
        if score < OCCUPIED_THRESHOLD:
            continue
        wx, wy = grid_to_world(gx, gy)
        sx, sy = world_to_screen(wx, wy, center_x, center_y, scale, view_x, view_y)
        if plot_area.collidepoint(sx, sy):
            size = max(2, int(MAP_RES_MM * scale))
            pygame.draw.rect(surface, (190, 90, 90), (sx - size // 2, sy - size // 2, size, size))
            map_drawn += 1

    for (gx, gy), cost in list(cost_grid.items()):
        if cost <= 0 or cost >= COST_OBSTACLE:
            continue
        wx, wy = grid_to_world(gx, gy)
        sx, sy = world_to_screen(wx, wy, center_x, center_y, scale, view_x, view_y)
        if plot_area.collidepoint(sx, sy):
            size = max(1, int(MAP_RES_MM * scale))
            shade = max(40, min(140, cost))
            pygame.draw.rect(surface, (shade, shade, 40), (sx - size // 2, sy - size // 2, size, size), 1)

    # Draw planned path
    if planned_path:
        screen_points = [world_to_screen(x, y, center_x, center_y, scale, view_x, view_y) for x, y in planned_path]
        visible = [pt for pt in screen_points if lidar_rect.collidepoint(pt[0], pt[1])]
        if len(screen_points) >= 2:
            pygame.draw.lines(surface, (0, 180, 255), False, screen_points, 2)
        for x, y in path_waypoints:
            px, py = world_to_screen(x, y, center_x, center_y, scale, view_x, view_y)
            if plot_area.collidepoint(px, py):
                pygame.draw.circle(surface, (0, 220, 255), (px, py), 4)

    # Draw queued target waypoints in world frame
    for x, y, _ in waypoints:
        px, py = world_to_screen(x, y, center_x, center_y, scale, view_x, view_y)
        if plot_area.collidepoint(px, py):
            pygame.draw.circle(surface, (255, 180, 0), (px, py), 5)

    # Draw LiDAR points transformed into world frame using Arduino pose
    now = time.time()
    visible_points = 0
    for _, data in list(latest_lidar_points.items()):
        angle_deg, distance_mm, intensity, timestamp = data
        age = now - timestamp
        if age > 1.0:
            continue

        local_x, local_y = lidar_to_local_xy(angle_deg, distance_mm)
        world_x, world_y = local_to_world(local_x, local_y)
        sx, sy = world_to_screen(world_x, world_y, center_x, center_y, scale, view_x, view_y)

        if plot_area.collidepoint(sx, sy):
            brightness = max(80, min(255, intensity * 3))
            pygame.draw.circle(surface, (80, brightness, 120), (sx, sy), 2)
            visible_points += 1

    # LiDAR odometry verification marker.
    # This shows where scan matching thinks the robot centre/yaw is, relative to
    # the encoder/IMU pose that stays centred on screen.
    if LIDAR_ODOM_ENABLED and lidar_odom_confidence > 0.05:
        vx, vy = world_to_screen(lidar_odom_x, lidar_odom_y, center_x, center_y, scale, view_x, view_y)
        if plot_area.collidepoint(vx, vy):
            marker_color = (180, 120, 255) if lidar_odom_confidence < 0.65 else (120, 220, 255)
            pygame.draw.circle(surface, marker_color, (vx, vy), 8, 2)
            pygame.draw.line(surface, marker_color, (vx - 8, vy), (vx + 8, vy), 1)
            pygame.draw.line(surface, marker_color, (vx, vy - 8), (vx, vy + 8), 1)
            vh = math.radians(lidar_odom_heading)
            vhx = vx + int(math.sin(vh) * 24)
            vhy = vy - int(math.cos(vh) * 24)
            pygame.draw.line(surface, marker_color, (vx, vy), (vhx, vhy), 2)

    # Robot marker moves inside the fixed map.
    if plot_area.collidepoint(robot_sx, robot_sy):
        pygame.draw.circle(surface, robot_marker_color(), (robot_sx, robot_sy), 7)
        heading_rad = math.radians(nav_h)
        hx = robot_sx + int(math.sin(heading_rad) * 26)
        hy = robot_sy - int(math.cos(heading_rad) * 26)
        pygame.draw.line(surface, robot_marker_color(), (robot_sx, robot_sy), (hx, hy), 2)
        pygame.draw.polygon(
            surface,
            robot_marker_color(),
            [(hx, hy), (hx - 5, hy + 8), (hx + 5, hy + 8)],
        )
    else:
        draw_text_fit(surface, "Robot is outside current map view - press DOWN to zoom out", tiny_font, (255, 160, 80), (plot_area.x + 10, plot_area.y + 8), plot_area.width - 20)

    arduino_ok = (time.time() - last_arduino_time) < 1.0 if last_arduino_time else False
    lidar_ok = (time.time() - last_lidar_time) < 1.0 if last_lidar_time else False

    pose_line = (f"Arduino X={robot_x:.1f} Y={robot_y:.1f} H={robot_heading:.2f}°  |  "
                 f"SLAM X={nav_x:.1f} Y={nav_y:.1f} H={nav_h:.2f}°")
    health_line = (
        f"Arduino: {'OK' if arduino_ok else 'No data'}   |   "
        f"LiDAR: {'OK' if lidar_ok else 'No data'}   |   "
        f"Range={max_range_m:.1f} m   Scan={visible_points} pts   FPS={fps:.1f}   |   {mapping_status}"
    )
    map_line = (
        f"Map cells={len(occupancy_grid)}   Path={len(path_waypoints)}/10   "
        f"Frame R={ROBOT_RADIUS_MM}mm   Stop clearance={FRONT_STOP_CLEARANCE_MM}mm   Centre trigger={FRONT_STOP_DISTANCE_MM}mm"
    )
    lidar_odom_line = (
        f"SLAM: {slam_status}   |   LiDAR odom={'ON' if LIDAR_ODOM_ENABLED else 'OFF'}   "
        f"ΔX={lidar_odom_error_x:+.0f}mm ΔY={lidar_odom_error_y:+.0f}mm "
        f"ΔYaw={lidar_odom_error_heading:+.1f}° conf={lidar_odom_confidence:.2f}"
    )

    if front_obstacle_distance_mm is None:
        stop_status = f"Front stop ON: stop when frame clearance < {FRONT_STOP_CLEARANCE_MM} mm | no front object"
        stop_color = muted_text_color()
    elif obstacle_detected:
        stop_status = f"EMERGENCY STOP: frame clearance {front_obstacle_clearance_mm:.0f} mm < {FRONT_STOP_CLEARANCE_MM} mm limit"
        stop_color = (220, 40, 40)
    else:
        stop_status = f"Front stop ON: frame clearance {front_obstacle_clearance_mm:.0f} mm | limit {FRONT_STOP_CLEARANCE_MM} mm"
        stop_color = muted_text_color()

    help_line = "P plan | V LiDAR odom | Q/E heading | R reset LiDAR | M map | C scan | S stop | U reset all"

    info_y = lidar_rect.bottom - 104
    info_x = lidar_rect.x + 12
    info_w = lidar_rect.width - 24
    draw_text_fit(surface, pose_line, tiny_font, main_text_color(), (info_x, info_y), info_w)
    draw_text_fit(surface, health_line, tiny_font, muted_text_color(), (info_x, info_y + 17), info_w)
    draw_text_fit(surface, map_line, tiny_font, highlight_text_color(), (info_x, info_y + 34), info_w)
    odom_color = ((0, 100, 160) if WHITE_BACKGROUND else (120, 220, 255)) if lidar_odom_confidence >= 0.65 else serial_warning_color()
    draw_text_fit(surface, lidar_odom_line, tiny_font, odom_color, (info_x, info_y + 51), info_w)
    draw_text_fit(surface, stop_status, tiny_font, stop_color, (info_x, info_y + 68), info_w)
    draw_text_fit(surface, help_line, tiny_font, muted_text_color(), (info_x, info_y + 85), info_w)


def draw_arduino_data_panel(surface):
    """Show more live data coming from Arduino."""
    panel_x = 25
    panel_y = 500 if HEIGHT >= 720 else 462
    panel_w = LEFT_PANEL_W - 40
    panel_h = max(70, status_rect.y - panel_y - 10)

    if panel_h < 55:
        return

    pygame.draw.rect(surface, serial_panel_color(), (panel_x, panel_y, panel_w, panel_h))
    pygame.draw.rect(surface, serial_panel_border_color(), (panel_x, panel_y, panel_w, panel_h), 1)

    arduino_ok = (time.time() - last_arduino_time) < 1.0 if last_arduino_time else False
    header_color = serial_ok_color() if arduino_ok else serial_warning_color()
    header = tiny_font.render(f"Arduino data  |  {'OK' if arduino_ok else 'No pose'}  |  lines={arduino_rx_count}", True, header_color)
    surface.blit(header, (panel_x + 8, panel_y + 8))

    pose_lines = [
        f"Enc X: {robot_x:.1f} mm",
        f"Enc Y: {robot_y:.1f} mm",
        f"IMU Yaw: {robot_heading:.2f} deg",
        f"LiDAR X: {lidar_odom_x:.1f} mm",
        f"LiDAR Y: {lidar_odom_y:.1f} mm",
        f"LiDAR Yaw: {lidar_odom_heading:.2f} deg",
        f"SLAM X/Y: {slam_x:.1f}, {slam_y:.1f} mm",
        f"SLAM Yaw: {slam_heading:.2f} deg",
        f"Error: {lidar_odom_error_dist:.0f} mm, {lidar_odom_error_heading:+.1f} deg",
    ]
    for i, line in enumerate(pose_lines):
        txt = tiny_font.render(line, True, main_text_color())
        surface.blit(txt, (panel_x + 8, panel_y + 28 + i * 15))

    log_y = panel_y + 138
    if log_y < panel_y + panel_h - 18:
        title = tiny_font.render("Recent raw serial:", True, muted_text_color())
        surface.blit(title, (panel_x + 8, log_y))
        log_y += 16

    max_lines = max(1, int((panel_y + panel_h - log_y - 4) / 14))
    max_chars = max(24, int((panel_w - 18) / 7))

    for i, line in enumerate(list(serial_log)[-max_lines:]):
        shown = line[:max_chars] + ("..." if len(line) > max_chars else "")
        color = serial_ok_color() if ">" in line and line.count(">") == 2 else muted_text_color()
        txt = tiny_font.render(shown, True, color)
        surface.blit(txt, (panel_x + 8, log_y + i * 14))


# Bottom-right raw serial log removed.

# =========================
# MAIN LOOP
# =========================
running = True

try:
    while running:
        read_arduino_lines()
        read_lidar_data()
        update_lidar_odometry_verification()
        update_slam_pose()
        update_mapping_start_gate()
        update_occupancy_map_from_lidar()
        update_front_obstacle_stop()

        fps = clock.get_fps()
        screen.fill(background_color())

        title = font.render("Robot Serial Controller", True, main_text_color())
        screen.blit(title, (25, 25))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.VIDEORESIZE and not fullscreen:
                WIDTH = max(MIN_WIDTH, event.w)
                HEIGHT = max(MIN_HEIGHT, event.h)
                WINDOWED_WIDTH, WINDOWED_HEIGHT = WIDTH, HEIGHT
                screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
                update_layout()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_F11:
                    toggle_fullscreen()
                elif event.key == pygame.K_c:
                    clear_lidar_map()
                elif event.key == pygame.K_m:
                    clear_fixed_map()
                elif event.key == pygame.K_p:
                    plan_path_to_input_target()
                elif event.key == pygame.K_s:
                    send_stop_command("manual key")
                elif event.key == pygame.K_u:
                    reset_arduino_uno()
                elif event.key == pygame.K_v:
                    toggle_lidar_odometry()
                elif event.key == pygame.K_b:
                    toggle_background_theme()
                elif event.key == pygame.K_q:
                    adjust_lidar_heading(-LIDAR_OFFSET_STEP_DEG)
                elif event.key == pygame.K_e:
                    adjust_lidar_heading(LIDAR_OFFSET_STEP_DEG)
                elif event.key == pygame.K_r:
                    reset_lidar_heading()
                elif event.key == pygame.K_UP:
                    max_range_m = max(1.0, max_range_m - 0.5)
                    last_message = f"LiDAR range set to {max_range_m:.1f} m"
                elif event.key == pygame.K_DOWN:
                    max_range_m = min(12.0, max_range_m + 0.5)
                    last_message = f"LiDAR range set to {max_range_m:.1f} m"

            for box in input_boxes:
                box.handle_event(event)

            if event.type == pygame.MOUSEBUTTONDOWN:
                for button in buttons:
                    if button.is_clicked(event.pos):
                        button.action()

        for box in input_boxes:
            box.draw(screen)

        for button in buttons:
            button.draw(screen)

        draw_waypoint_list(screen)
        draw_arduino_data_panel(screen)
        draw_lidar_screen(screen, fps)

        # Bottom status area. The command format is on its own row so it does not
        # overlap the Arduino status text.
        pygame.draw.rect(screen, status_panel_color(), status_rect)
        pygame.draw.rect(screen, status_border_color(), status_rect, 1)

        status_x = status_rect.x + 15
        status_w = status_rect.width - 30
        draw_text_fit(screen, f"Status: {last_message}", small_font, highlight_text_color(), (status_x, status_rect.y + 8), status_w)

        command_line = "Arduino command format: (X,Y,wait_sec):  |  B white/dark background  |  V LiDAR odom  |  U Reset All  |  F11 fullscreen/window  |  ESC quit"
        draw_text_fit(screen, command_line, tiny_font, muted_text_color(), (status_x, status_rect.y + 34), status_w)

        pygame.display.flip()
        clock.tick(60)

except KeyboardInterrupt:
    pass

finally:
    if lidar is not None:
        try:
            lidar.write(STOP_CMD)
            time.sleep(0.1)
            print("LiDAR stop command sent: AA 55 F5 0A")
        except Exception:
            pass
        lidar.close()

    if arduino is not None:
        arduino.close()

    pygame.quit()
