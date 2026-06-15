#!/usr/bin/env python3
"""
Pygame robot waypoint controller + Arduino odometry + COIN-D6 LiDAR map.

Arduino side:
- Serial.begin(500000)
- Command format: (X_mm,Y_mm,wait_seconds):
- Feedback pose format: X>Y>Heading

COIN-D6 LiDAR side:
- Default port: /dev/ttyUSB0
- Default baud: 230400
- Start command: AA 55 F0 0F
- Stop command:  AA 55 F5 0A

Keys:
- C = clear LiDAR map
- S = send emergency STOP to Arduino
- R = reset Arduino, LiDAR, and Pygame state
- F11 or F = toggle full screen / resizable window
- A = toggle automatic obstacle avoidance route planner
- UP = zoom in / reduce range
- DOWN = zoom out / increase range
- ESC or Q = quit

Safety stop:
- If COIN-D6 detects an object in the front cone below 300 mm,
  Python sends STOP:, creates a 3-10 point detour route, and sends it to Arduino.

LiDAR calibration:
- Place a flat object directly in front of the robot.
- Type the real measured distance in mm, then press Cal LiDAR.
- Example: if the object is 100 mm away, enter 100 so 100 mm in Pygame matches 100 mm in real measurement.
"""

import math
import re
import time
from collections import deque

import pygame
import serial

# =========================
# SERIAL PORT SETUP
# =========================
# Change these two ports to match your computer/Raspberry Pi.
# Windows example: ARDUINO_PORT = "COM5", LIDAR_PORT = "COM6"
# Raspberry Pi / Ubuntu example: ARDUINO_PORT = "/dev/ttyACM0", LIDAR_PORT = "/dev/ttyUSB0"
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

# Start in full screen by default. Press F11 or F to toggle back to a resizable window.
WINDOWED_SIZE = (1100, 650)
MIN_WIDTH, MIN_HEIGHT = 900, 500
fullscreen_mode = True


def set_display_mode(fullscreen):
    """Create the Pygame display in full screen or resizable window mode."""
    global screen, WIDTH, HEIGHT, fullscreen_mode

    fullscreen_mode = fullscreen
    if fullscreen_mode:
        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    else:
        screen = pygame.display.set_mode(WINDOWED_SIZE, pygame.RESIZABLE)

    WIDTH, HEIGHT = screen.get_size()
    pygame.display.set_caption("Robot Controller + COIN-D6 LiDAR Map")
    update_layout()


def toggle_fullscreen():
    """Switch between full screen and a resizable window."""
    set_display_mode(not fullscreen_mode)


def update_layout():
    """Recalculate map and status-panel positions after resize/fullscreen changes."""
    global WIDTH, HEIGHT, lidar_rect, status_rect

    WIDTH, HEIGHT = screen.get_size()
    margin = 25
    status_h = 58
    control_width = 330

    status_rect = pygame.Rect(
        margin,
        max(MIN_HEIGHT - status_h - margin, HEIGHT - status_h - margin),
        max(200, WIDTH - margin * 2),
        status_h,
    )

    # Keep the map box clean and centred in the available right-side space.
    # Text is drawn in a header/footer inside this box, while the LiDAR scan uses
    # a separate inner plot area that is centred inside the remaining space.
    map_x = control_width + margin
    map_y = margin
    map_w = max(360, WIDTH - map_x - margin)
    map_h = max(280, status_rect.y - map_y - margin)
    lidar_rect = pygame.Rect(map_x, map_y, map_w, map_h)


# Temporary rectangles are overwritten by set_display_mode().
lidar_rect = pygame.Rect(350, 45, 520, 370)
status_rect = pygame.Rect(25, 435, 845, 45)
WIDTH, HEIGHT = WINDOWED_SIZE
screen = None

font = pygame.font.SysFont("Arial", 22)
small_font = pygame.font.SysFont("Arial", 17)
tiny_font = pygame.font.SysFont("Arial", 14)
clock = pygame.time.Clock()
set_display_mode(True)

# White-background UI colours
BG = (211, 211, 211)
PANEL_BG = (211, 211, 211)
MAP_BG = (211, 211, 211)
TEXT = (25, 25, 25)
TEXT_MUTED = (95, 95, 95)
BORDER = (185, 185, 185)
GRID = (190, 190, 190)
AXIS = (190, 190, 190)
GREEN = (0, 135, 80)
GRAY = (128, 128, 128)
BLUE = (0, 125, 220)
RED = (220, 60, 60)
ORANGE = (230, 145, 0)

last_message = "Waiting for command..."
serial_log = deque(maxlen=7)

# Arduino pose from X>Y>Heading
robot_x = 0.0
robot_y = 0.0
robot_heading = 0.0
last_arduino_time = 0.0
arduino_line_buffer = bytearray()

# LiDAR data
lidar_buffer = bytearray()
latest_lidar_points = {}   # angle_bin: (angle_deg, calibrated_mm, raw_mm, intensity, timestamp)
lidar_packet_count = 0
last_lidar_time = 0.0
max_range_m = 4.0

# Robot frame drawn on the map, in real millimetres.
ROBOT_FRAME_WIDTH_MM = 200.0
ROBOT_FRAME_LENGTH_MM = 200.0

# LiDAR distance calibration. Default is raw distance = real distance.
# Press Cal LiDAR after placing an object at a known distance in front of the robot.
LIDAR_DISTANCE_SCALE = 1.0
LIDAR_DISTANCE_OFFSET_MM = 0.0

# Front obstacle safety stop
AUTO_STOP_ENABLED = True
FRONT_STOP_DISTANCE_MM = 300
FRONT_CONE_HALF_ANGLE_DEG = 45  # half-angle; 45 deg each side = 90 deg total front stop cone
STOP_REPEAT_SECONDS = 0.30
obstacle_detected = False
front_obstacle_distance_mm = None
last_stop_command_time = 0.0

# Stored waypoint queue: [(x_mm, y_mm, wait_sec), ...]
waypoints = []

# Automatic obstacle avoidance route planner.
# When an obstacle enters the emergency zone, Python sends STOP, creates a local
# detour path around the obstacle, then sends the new 3-10 point route to Arduino.
AUTO_AVOIDANCE_ENABLED = True
AVOID_MIN_POINTS = 3
AVOID_MAX_POINTS = 10
AVOID_REPLAN_COOLDOWN_SECONDS = 5.0
AVOID_TARGET_REACHED_MM = 180.0
AVOID_LATERAL_STEP_MM = 650.0
AVOID_FRONT_PASS_EXTRA_MM = 750.0
AVOID_FORWARD_START_MM = 150.0
AVOID_SIDE_SCAN_MIN_DEG = 25.0
AVOID_SIDE_SCAN_MAX_DEG = 115.0
last_avoidance_plan_time = 0.0
last_avoidance_pose = (0.0, 0.0)
avoidance_active = False
avoidance_route = []
mission_waypoints = []

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
        label_surface = tiny_font.render(self.label, True, TEXT_MUTED)
        surface.blit(label_surface, (self.rect.x, self.rect.y - 18))

        color = BLUE if self.active else BORDER
        pygame.draw.rect(surface, PANEL_BG, self.rect)
        pygame.draw.rect(surface, color, self.rect, 2)

        text_surface = small_font.render(self.text, True, TEXT)
        surface.blit(text_surface, (self.rect.x + 8, self.rect.y + 7))

# =========================
# BUTTON CLASS
# =========================
class Button:
    def __init__(self, x, y, w, h, text, action):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.action = action

    def is_hovered(self, pos=None):
        if pos is None:
            pos = pygame.mouse.get_pos()
        return self.rect.collidepoint(pos)

    def draw(self, surface):
        hovered = self.is_hovered()

        fill = (235, 235, 235)
        hover_fill = (220, 220, 220)
        border_colour = BORDER
        text_colour = TEXT

        if self.text == "STOP":
            fill = (255, 230, 230)
            hover_fill = (255, 205, 205)
            text_colour = RED
        elif self.text == "Reset All":
            fill = (255, 241, 214)
            hover_fill = (255, 225, 170)
            text_colour = (140, 80, 0)
        elif self.text == "Cal LiDAR":
            fill = (225, 243, 255)
            hover_fill = (200, 232, 255)
            text_colour = BLUE

        if hovered:
            fill = hover_fill
            border_colour = BLUE

        # Slight drop shadow makes the hovered button feel clickable.
        if hovered:
            shadow_rect = self.rect.move(1, 2)
            pygame.draw.rect(surface, (220, 220, 220), shadow_rect, border_radius=5)

        pygame.draw.rect(surface, fill, self.rect, border_radius=5)
        pygame.draw.rect(surface, border_colour, self.rect, 2 if hovered else 1, border_radius=5)

        text_surface = tiny_font.render(self.text, True, text_colour)
        text_rect = text_surface.get_rect(center=self.rect.center)
        surface.blit(text_surface, text_rect)

    def is_clicked(self, pos):
        return self.is_hovered(pos)


def draw_text_fit(surface, text, font_obj, color, x, y, max_width):
    """Draw one line of text and shorten it safely so it does not overlap nearby UI."""
    if max_width <= 8:
        return

    rendered = font_obj.render(text, True, color)
    if rendered.get_width() <= max_width:
        surface.blit(rendered, (x, y))
        return

    suffix = "..."
    low, high = 0, len(text)
    while low < high:
        mid = (low + high + 1) // 2
        candidate = text[:mid].rstrip() + suffix
        if font_obj.size(candidate)[0] <= max_width:
            low = mid
        else:
            high = mid - 1

    shortened = text[:low].rstrip() + suffix
    surface.blit(font_obj.render(shortened, True, color), (x, y))

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


def calibrate_lidar_distance(raw_distance_mm):
    """Convert raw LiDAR distance to real-world mm using the current calibration scale."""
    corrected = raw_distance_mm * LIDAR_DISTANCE_SCALE + LIDAR_DISTANCE_OFFSET_MM
    return max(0.0, corrected)


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

            for angle_deg, raw_distance_mm, intensity, _ in points:
                distance_mm = calibrate_lidar_distance(raw_distance_mm)
                if distance_mm < 50:
                    continue
                if distance_mm > max_range_m * 1000:
                    continue

                # 0.5-degree bins keep the newest point for each direction.
                angle_key = round(angle_deg * 2) / 2.0
                latest_lidar_points[angle_key] = (angle_deg, distance_mm, raw_distance_mm, intensity, now)

    except Exception as e:
        serial_log.append(f"LiDAR read failed: {e}")

# =========================
# COORDINATE TRANSFORMS
# =========================
def lidar_to_local_xy(angle_deg, distance_mm):
    """
    COIN-D6 frame:
    0 deg = forward/up, 90 deg = right, angle increases clockwise.
    Local robot frame:
    x = right, y = forward.
    """
    theta = math.radians(angle_deg)
    local_x = distance_mm * math.sin(theta)
    local_y = distance_mm * math.cos(theta)
    return local_x, local_y


def local_to_world(local_x, local_y):
    """
    Uses Arduino odometry frame:
    heading 0 deg means robot faces +Y.
    Positive heading turns toward +X.
    """
    th = math.radians(robot_heading)
    world_x = robot_x + local_x * math.cos(th) + local_y * math.sin(th)
    world_y = robot_y - local_x * math.sin(th) + local_y * math.cos(th)
    return world_x, world_y


def world_to_screen(x_mm, y_mm, center_x, center_y, scale, view_x=0.0, view_y=0.0):
    sx = center_x + int((x_mm - view_x) * scale)
    sy = center_y - int((y_mm - view_y) * scale)
    return sx, sy

# =========================
# ARDUINO SERIAL FUNCTIONS
# =========================
def clean_waypoint_list(points, max_points=AVOID_MAX_POINTS):
    """Round, limit, and remove duplicate consecutive waypoints."""
    cleaned = []
    for x, y, wait_sec in points:
        item = (int(round(x)), int(round(y)), max(0, int(round(wait_sec))))
        if cleaned and abs(cleaned[-1][0] - item[0]) < 5 and abs(cleaned[-1][1] - item[1]) < 5:
            continue
        cleaned.append(item)
        if len(cleaned) >= max_points:
            break
    return cleaned


def build_command_from_points(points):
    route = clean_waypoint_list(points)
    if not route:
        return None, []
    return " ".join(f"({x},{y},{wait_sec})" for x, y, wait_sec in route) + ":\n", route


def build_command():
    command, _route = build_command_from_points(waypoints)
    return command


def send_waypoint_list(points, label="Route", replace_queue=False):
    """Send any waypoint list to Arduino using the same (X,Y,wait): protocol."""
    global last_message, mission_waypoints

    command, route = build_command_from_points(points)
    if command is None:
        last_message = "No waypoint in route"
        return False

    if len(route) > AVOID_MAX_POINTS:
        route = route[:AVOID_MAX_POINTS]

    if arduino is None:
        last_message = "Arduino not connected"
        print(last_message)
        return False

    try:
        arduino.write(command.encode("utf-8"))
        arduino.flush()

        if replace_queue:
            waypoints[:] = route

        mission_waypoints = list(route)
        last_message = f"{label} sent: {len(route)} point(s) -> {command.strip()}"
        serial_log.append(last_message)
        print(last_message)
        return True
    except Exception as e:
        last_message = f"Serial send failed: {e}"
        serial_log.append(last_message)
        print(last_message)
        return False


def send_waypoints():
    if not waypoints:
        global last_message
        last_message = "No waypoint in queue"
        return
    send_waypoint_list(waypoints, label="Manual route", replace_queue=False)


def read_arduino_lines():
    """Read Arduino feedback without blocking the Pygame screen."""
    global last_message, robot_x, robot_y, robot_heading, last_arduino_time

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



def distance_between_points_mm(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def local_offset_to_world(local_x, local_y):
    """Convert a local robot-frame offset into a world waypoint."""
    th = math.radians(robot_heading)
    world_x = robot_x + local_x * math.cos(th) + local_y * math.sin(th)
    world_y = robot_y - local_x * math.sin(th) + local_y * math.cos(th)
    return world_x, world_y


def world_to_local_from_robot(world_x, world_y):
    """Convert a world position into the current local robot frame."""
    dx = world_x - robot_x
    dy = world_y - robot_y
    th = math.radians(robot_heading)
    local_x = dx * math.cos(th) - dy * math.sin(th)
    local_y = dx * math.sin(th) + dy * math.cos(th)
    return local_x, local_y


def current_mission_source():
    """Use the route currently being executed, or fall back to the visible queue."""
    if mission_waypoints:
        return mission_waypoints
    return waypoints


def find_current_goal_index():
    """Find the next waypoint that has not been reached yet."""
    source = current_mission_source()
    if not source:
        return None, None

    for index, (x, y, wait_sec) in enumerate(source):
        if distance_between_points_mm(robot_x, robot_y, x, y) > AVOID_TARGET_REACHED_MM:
            return index, (x, y, wait_sec)

    # If every point is already close, keep the final point as a safe fallback.
    return len(source) - 1, source[-1]


def nearest_distance_in_error_range(min_error_deg, max_error_deg, max_age_s=0.45):
    """Return nearest calibrated LiDAR distance in a signed front-angle band."""
    now = time.time()
    nearest = None
    for angle_deg, distance_mm, _raw_distance_mm, _intensity, timestamp in list(latest_lidar_points.values()):
        if now - timestamp > max_age_s or distance_mm <= 0:
            continue
        err = front_angle_error(angle_deg)
        if min_error_deg <= err <= max_error_deg:
            if nearest is None or distance_mm < nearest:
                nearest = distance_mm
    return nearest


def choose_avoidance_side():
    """
    Pick the safer side from the LiDAR scan.
    Local X is positive to the right and negative to the left.
    """
    left_nearest = nearest_distance_in_error_range(-AVOID_SIDE_SCAN_MAX_DEG, -AVOID_SIDE_SCAN_MIN_DEG)
    right_nearest = nearest_distance_in_error_range(AVOID_SIDE_SCAN_MIN_DEG, AVOID_SIDE_SCAN_MAX_DEG)

    # Treat no return as open space up to the current LiDAR range.
    open_distance = max_range_m * 1000.0
    left_score = left_nearest if left_nearest is not None else open_distance
    right_score = right_nearest if right_nearest is not None else open_distance

    # Default to left when both sides look similar, matching the hallway example.
    side_sign = -1 if left_score >= right_score else 1
    side_name = "left" if side_sign < 0 else "right"
    return side_sign, side_name, left_score, right_score


def build_avoidance_route(nearest_front_mm):
    """
    Create a 3-10 point detour around the front obstacle.
    The route is a simple local pathfinding result: side-step, move past the
    obstacle, return to the original line, then continue to the current goal.
    """
    goal_index, current_goal = find_current_goal_index()
    if current_goal is None:
        return [], "No active waypoint to continue after obstacle"

    source = current_mission_source()
    goal_x, goal_y, goal_wait = current_goal
    goal_local_x, goal_local_y = world_to_local_from_robot(goal_x, goal_y)

    side_sign, side_name, left_score, right_score = choose_avoidance_side()

    # Keep the lateral offset large enough for the 200x200 mm frame plus margin.
    lateral = max(AVOID_LATERAL_STEP_MM, ROBOT_FRAME_WIDTH_MM + 350.0)

    obstacle_forward = max(FRONT_STOP_DISTANCE_MM, nearest_front_mm or FRONT_STOP_DISTANCE_MM)
    pass_forward = obstacle_forward + AVOID_FRONT_PASS_EXTRA_MM

    # If the target is close, still create a useful triangle/rectangle route.
    if goal_local_y > AVOID_FORWARD_START_MM:
        pass_forward = min(max(pass_forward, AVOID_FORWARD_START_MM + 350.0), max(goal_local_y, pass_forward))

    first_forward = min(AVOID_FORWARD_START_MM, max(60.0, pass_forward * 0.25))

    p1 = local_offset_to_world(side_sign * lateral, first_forward)
    p2 = local_offset_to_world(side_sign * lateral, pass_forward)
    p3 = local_offset_to_world(0.0, pass_forward)

    detour = [
        (p1[0], p1[1], 0),
        (p2[0], p2[1], 0),
        (p3[0], p3[1], 0),
        (goal_x, goal_y, goal_wait),
    ]

    # Continue with the remaining original route after the current goal.
    remaining = list(source[goal_index + 1:]) if goal_index is not None else []
    route = clean_waypoint_list(detour + remaining, max_points=AVOID_MAX_POINTS)

    # Make sure the route always has at least 3 points for the Arduino path.
    while len(route) < AVOID_MIN_POINTS:
        extra_forward = pass_forward + 250.0 * (len(route) + 1)
        extra = local_offset_to_world(0.0, extra_forward)
        route.append((int(round(extra[0])), int(round(extra[1])), 0))

    message = (
        f"Avoidance path: {side_name}, {len(route)} point(s), "
        f"L={left_score:.0f}mm R={right_score:.0f}mm"
    )
    return route[:AVOID_MAX_POINTS], message


def handle_obstacle_avoidance(nearest_front_mm):
    """Stop once, calculate a detour, and automatically send it to Arduino."""
    global last_avoidance_plan_time, last_avoidance_pose, avoidance_active, avoidance_route, last_message

    if not AUTO_AVOIDANCE_ENABLED:
        return

    now = time.time()
    if now - last_avoidance_plan_time < AVOID_REPLAN_COOLDOWN_SECONDS:
        return

    send_stop_command(f"front obstacle {nearest_front_mm:.0f} mm - replanning")
    last_avoidance_plan_time = now
    last_avoidance_pose = (robot_x, robot_y)

    route, message = build_avoidance_route(nearest_front_mm)
    if not route:
        last_message = f"Obstacle stop: {message}"
        serial_log.append(last_message)
        return

    avoidance_route = list(route)
    avoidance_active = True

    # Give Arduino a short moment to process STOP before receiving the new route.
    time.sleep(0.05)
    sent = send_waypoint_list(route, label="Auto avoidance route", replace_queue=True)
    if sent:
        last_message = message + " | sent to Arduino"
    else:
        last_message = message + " | send failed"
    serial_log.append(last_message)

def front_angle_error(angle_deg):
    """Return signed angle difference from robot front, where 0 deg is forward."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def update_front_obstacle_stop():
    """Check live LiDAR points in front of the robot and stop if anything is too close."""
    global obstacle_detected, front_obstacle_distance_mm

    if not AUTO_STOP_ENABLED:
        obstacle_detected = False
        front_obstacle_distance_mm = None
        return

    now = time.time()
    nearest = None

    for angle_deg, distance_mm, _raw_distance_mm, _intensity, timestamp in list(latest_lidar_points.values()):
        if now - timestamp > 0.35:
            continue

        if distance_mm <= 0:
            continue

        if abs(front_angle_error(angle_deg)) <= FRONT_CONE_HALF_ANGLE_DEG:
            if nearest is None or distance_mm < nearest:
                nearest = distance_mm

    front_obstacle_distance_mm = nearest
    obstacle_detected = nearest is not None and nearest < FRONT_STOP_DISTANCE_MM

    if obstacle_detected:
        if AUTO_AVOIDANCE_ENABLED:
            handle_obstacle_avoidance(nearest)
        elif (now - last_stop_command_time) >= STOP_REPEAT_SECONDS:
            send_stop_command(f"front obstacle {nearest:.0f} mm")

# =========================
# UI ACTIONS
# =========================
x_box = InputBox(25, 70, 80, 35, "X mm", "")
y_box = InputBox(120, 70, 80, 35, "Y mm", "")
wait_box = InputBox(215, 70, 100, 35, "Wait sec", "")
calibration_distance_box = InputBox(25, 215, 90, 32, "Real mm", "100")
input_boxes = [x_box, y_box, wait_box, calibration_distance_box]


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
    x = x_box.value_int(0)
    y = y_box.value_int(0)
    wait_sec = max(0, wait_box.value_int(0))
    send_waypoint_list([(x, y, wait_sec)], label="Single waypoint", replace_queue=False)


def clear_waypoints():
    global last_message, avoidance_active
    waypoints.clear()
    mission_waypoints.clear()
    avoidance_route.clear()
    avoidance_active = False
    last_message = "Waypoint queue cleared"


def clear_lidar_map():
    global last_message
    latest_lidar_points.clear()
    last_message = "LiDAR map cleared"


def manual_stop():
    send_stop_command("manual button")


def nearest_front_raw_lidar_distance(max_age_s=0.7):
    """Return the nearest raw LiDAR distance in the front cone for calibration."""
    now = time.time()
    nearest = None

    for angle_deg, _distance_mm, raw_distance_mm, _intensity, timestamp in list(latest_lidar_points.values()):
        if now - timestamp > max_age_s:
            continue
        if raw_distance_mm <= 0:
            continue
        if abs(front_angle_error(angle_deg)) <= FRONT_CONE_HALF_ANGLE_DEG:
            if nearest is None or raw_distance_mm < nearest:
                nearest = raw_distance_mm

    return nearest


def calibrate_lidar_now():
    """Use the nearest front LiDAR point and a known measured distance to set the scale."""
    global LIDAR_DISTANCE_SCALE, last_message

    real_distance_mm = max(1, calibration_distance_box.value_int(100))
    raw_distance_mm = nearest_front_raw_lidar_distance()

    if raw_distance_mm is None:
        last_message = "Cal failed: place a target directly in front of LiDAR, then press Cal LiDAR"
        serial_log.append(last_message)
        return

    LIDAR_DISTANCE_SCALE = real_distance_mm / raw_distance_mm
    latest_lidar_points.clear()
    last_message = (
        f"LiDAR calibrated: raw {raw_distance_mm:.1f} mm -> real {real_distance_mm} mm "
        f"scale={LIDAR_DISTANCE_SCALE:.4f}"
    )
    serial_log.append(last_message)


def restart_lidar():
    """Stop, clear, and restart COIN-D6 streaming."""
    global last_lidar_time, lidar_packet_count

    lidar_buffer.clear()
    latest_lidar_points.clear()
    lidar_packet_count = 0
    last_lidar_time = 0.0

    if lidar is None:
        serial_log.append("LiDAR reset skipped: not connected")
        return

    try:
        lidar.write(STOP_CMD)
        lidar.flush()
        time.sleep(0.05)
        lidar.reset_input_buffer()
        lidar.write(START_CMD)
        lidar.flush()
        serial_log.append("LiDAR restarted")
    except Exception as e:
        serial_log.append(f"LiDAR reset failed: {e}")


def reset_arduino_serial():
    """Reset Arduino through serial if connected. Also sends RESET: for sketches that support it."""
    global last_arduino_time

    last_arduino_time = 0.0
    arduino_line_buffer.clear()

    if arduino is None:
        serial_log.append("Arduino reset skipped: not connected")
        return

    try:
        arduino.write(b"STOP:\n")
        arduino.write(b"RESET:\n")
        arduino.flush()

        # USB serial DTR toggle resets most Arduino Uno/Mega boards.
        try:
            arduino.setDTR(False)
            time.sleep(0.10)
            arduino.setDTR(True)
            time.sleep(1.80)
        except Exception:
            pass

        arduino.reset_input_buffer()
        try:
            arduino.reset_output_buffer()
        except Exception:
            pass
        serial_log.append("Arduino reset command sent")
    except Exception as e:
        serial_log.append(f"Arduino reset failed: {e}")


def reset_pygame_state():
    """Reset only the Python/Pygame-side state while keeping the current calibration scale."""
    global robot_x, robot_y, robot_heading, obstacle_detected, front_obstacle_distance_mm
    global last_stop_command_time, last_message, last_avoidance_plan_time, avoidance_active

    robot_x = 0.0
    robot_y = 0.0
    robot_heading = 0.0
    obstacle_detected = False
    front_obstacle_distance_mm = None
    last_stop_command_time = 0.0
    last_avoidance_plan_time = 0.0
    avoidance_active = False

    waypoints.clear()
    mission_waypoints.clear()
    avoidance_route.clear()
    latest_lidar_points.clear()
    lidar_buffer.clear()
    arduino_line_buffer.clear()
    serial_log.clear()

    x_box.text = "0"
    y_box.text = "1000"
    wait_box.text = "0"
    last_message = "Pygame state reset"


def reset_all():
    """Reset Arduino, LiDAR, and Pygame-side values from one button."""
    global last_message

    reset_pygame_state()
    restart_lidar()
    reset_arduino_serial()
    last_message = "Reset All completed: Pygame cleared, LiDAR restarted, Arduino reset sent"
    serial_log.append(last_message)


buttons = [
    Button(25, 125, 90, 32, "Add", add_waypoint),
    Button(125, 125, 90, 32, "Send One", send_single_now),
    Button(225, 125, 90, 32, "Send All", send_waypoints),
    Button(25, 165, 90, 30, "STOP", manual_stop),
    Button(125, 165, 90, 30, "Clear WP", clear_waypoints),
    Button(225, 165, 90, 30, "Clear LiDAR", clear_lidar_map),
    Button(125, 215, 90, 32, "Cal LiDAR", calibrate_lidar_now),
    Button(225, 215, 90, 32, "Reset All", reset_all),
]

def update_mouse_cursor():
    """Show a hand cursor when the mouse is over any clickable button."""
    try:
        if any(button.is_hovered() for button in buttons):
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_HAND)
        else:
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
    except Exception:
        # Some older SDL/Pygame builds do not support system cursors.
        pass

# =========================
# DRAWING FUNCTIONS
# =========================
def draw_waypoint_list(surface):
    hint_y = 255
    draw_text_fit(
        surface,
        "Calibration: place target in front, enter real mm, press Cal LiDAR",
        tiny_font,
        TEXT_MUTED,
        25,
        hint_y,
        290,
    )

    title_y = hint_y + 34
    title = small_font.render("Waypoint Queue", True, TEXT)
    surface.blit(title, (25, title_y))

    available_bottom = status_rect.y - 115
    line_y = title_y + 28
    if line_y > available_bottom:
        return

    if not waypoints:
        draw_text_fit(surface, "No waypoints added", tiny_font, TEXT_MUTED, 25, line_y, 285)
        return

    max_lines = max(1, min(6, (available_bottom - line_y) // 22))
    start_index = max(0, len(waypoints) - max_lines)
    for row, (x, y, w) in enumerate(waypoints[start_index:], start=start_index + 1):
        yy = line_y + (row - start_index - 1) * 22
        draw_text_fit(surface, f"{row}. ({x}, {y}, {w})", tiny_font, TEXT, 25, yy, 285)


def draw_lidar_screen(surface, fps):
    pygame.draw.rect(surface, MAP_BG, lidar_rect)
    pygame.draw.rect(surface, GREEN, lidar_rect, 2)

    pad = 12
    header_h = 36
    footer_h = 78

    title = "COIN-D6 LiDAR + Arduino Odometry Map"
    draw_text_fit(surface, title, small_font, GREEN, lidar_rect.x + pad, lidar_rect.y + 10, lidar_rect.width - pad * 2)

    footer_top = lidar_rect.bottom - footer_h
    plot_rect = pygame.Rect(
        lidar_rect.x + pad,
        lidar_rect.y + header_h + pad,
        lidar_rect.width - pad * 2,
        max(80, footer_top - (lidar_rect.y + header_h + pad) - pad),
    )

    # This is the real centre of the LiDAR plotting box.
    center_x = plot_rect.centerx
    center_y = plot_rect.centery
    plot_radius = max(20, min(plot_rect.width, plot_rect.height) // 2 - 10)
    scale = plot_radius / (max_range_m * 1000.0)  # pixels per mm, calibrated real distance

    # Keep the view centred on the robot. Robot stays in the middle while LiDAR points rotate/move around it.
    view_x = robot_x
    view_y = robot_y

    # Light boundary for the actual scan area, so the plot centre is visually clear.
    pygame.draw.rect(surface, (238, 238, 238), plot_rect, 1)

    # Range rings: these are drawn from calibrated real millimetres.
    max_whole_m = int(max_range_m)
    for r_m in range(1, max_whole_m + 1):
        radius = int(r_m * 1000 * scale)
        pygame.draw.circle(surface, GRID, (center_x, center_y), radius, 1)
        label = f"{r_m}m"
        label_x = min(center_x + radius + 4, plot_rect.right - 32)
        label_y = max(plot_rect.top + 2, center_y - 8)
        draw_text_fit(surface, label, tiny_font, TEXT_MUTED, label_x, label_y, 32)

    # Axis lines
    pygame.draw.line(surface, AXIS, (center_x, center_y - plot_radius), (center_x, center_y + plot_radius), 1)
    pygame.draw.line(surface, AXIS, (center_x - plot_radius, center_y), (center_x + plot_radius, center_y), 1)

    # Front safety stop zone. Anything inside this cone below the limit triggers STOP.
    stop_radius = int(FRONT_STOP_DISTANCE_MM * scale)
    left_angle = math.radians(-FRONT_CONE_HALF_ANGLE_DEG)
    right_angle = math.radians(FRONT_CONE_HALF_ANGLE_DEG)
    left_pt = (center_x + int(math.sin(left_angle) * stop_radius), center_y - int(math.cos(left_angle) * stop_radius))
    right_pt = (center_x + int(math.sin(right_angle) * stop_radius), center_y - int(math.cos(right_angle) * stop_radius))
    zone_color = RED if obstacle_detected else (235, 170, 170)
    pygame.draw.line(surface, zone_color, (center_x, center_y), left_pt, 1)
    pygame.draw.line(surface, zone_color, (center_x, center_y), right_pt, 1)
    pygame.draw.arc(
        surface,
        zone_color,
        pygame.Rect(center_x - stop_radius, center_y - stop_radius, stop_radius * 2, stop_radius * 2),
        math.radians(90 - FRONT_CONE_HALF_ANGLE_DEG),
        math.radians(90 + FRONT_CONE_HALF_ANGLE_DEG),
        2,
    )

    old_clip = surface.get_clip()
    surface.set_clip(plot_rect)

    # Draw queued/active target waypoints in world frame.
    route_points_screen = []
    for x, y, _ in waypoints:
        px, py = world_to_screen(x, y, center_x, center_y, scale, view_x, view_y)
        route_points_screen.append((px, py))
        if plot_rect.collidepoint(px, py):
            pygame.draw.circle(surface, ORANGE, (px, py), 5)

    if len(route_points_screen) >= 2:
        for a, b in zip(route_points_screen, route_points_screen[1:]):
            pygame.draw.line(surface, ORANGE, a, b, 2)

    # Highlight the latest automatically generated avoidance route.
    if avoidance_route:
        avoid_screen = []
        for x, y, _ in avoidance_route:
            px, py = world_to_screen(x, y, center_x, center_y, scale, view_x, view_y)
            avoid_screen.append((px, py))
            if plot_rect.collidepoint(px, py):
                pygame.draw.circle(surface, BLUE, (px, py), 4)
        if len(avoid_screen) >= 2:
            for a, b in zip(avoid_screen, avoid_screen[1:]):
                pygame.draw.line(surface, BLUE, a, b, 2)

    # Draw LiDAR points transformed into world frame using Arduino pose.
    # Distances here have already been corrected by the LiDAR calibration scale.
    now = time.time()
    visible_points = 0
    for _, data in list(latest_lidar_points.items()):
        angle_deg, distance_mm, raw_distance_mm, intensity, timestamp = data
        age = now - timestamp
        if age > 1.0:
            continue

        local_x, local_y = lidar_to_local_xy(angle_deg, distance_mm)
        world_x, world_y = local_to_world(local_x, local_y)
        sx, sy = world_to_screen(world_x, world_y, center_x, center_y, scale, view_x, view_y)

        if plot_rect.collidepoint(sx, sy):
            brightness = max(80, min(255, intensity * 3))
            point_colour = (40, min(170, brightness), 90)
            pygame.draw.circle(surface, point_colour, (sx, sy), 2)
            visible_points += 1

    # Robot frame: 200 mm x 200 mm in calibrated map scale.
    half_w = ROBOT_FRAME_WIDTH_MM / 2.0
    half_l = ROBOT_FRAME_LENGTH_MM / 2.0
    corners_local = [
        (-half_w, half_l),   # front-left
        (half_w, half_l),    # front-right
        (half_w, -half_l),   # rear-right
        (-half_w, -half_l),  # rear-left
    ]
    heading_rad = math.radians(robot_heading)
    robot_corners = []
    for local_x, local_y in corners_local:
        world_dx = local_x * math.cos(heading_rad) + local_y * math.sin(heading_rad)
        world_dy = -local_x * math.sin(heading_rad) + local_y * math.cos(heading_rad)
        sx = center_x + int(world_dx * scale)
        sy = center_y - int(world_dy * scale)
        robot_corners.append((sx, sy))

    pygame.draw.polygon(surface, TEXT, robot_corners, 2)
    # Mark only the front edge of the 200 mm frame. No triangle or long line is drawn in front of the robot.
    pygame.draw.line(surface, BLUE, robot_corners[0], robot_corners[1], 3)
    pygame.draw.circle(surface, TEXT, (center_x, center_y), 4)

    surface.set_clip(old_clip)

    arduino_ok = (time.time() - last_arduino_time) < 1.0 if last_arduino_time else False
    lidar_ok = (time.time() - last_lidar_time) < 1.0 if last_lidar_time else False

    pose_text = (
        f"Robot X={robot_x:.1f} mm  Y={robot_y:.1f} mm  H={robot_heading:.2f} deg  "
        f"Frame=200x200 mm"
    )
    info_text = (
        f"Arduino {'OK' if arduino_ok else 'No data'} | LiDAR {'OK' if lidar_ok else 'No data'} | "
        f"Range={max_range_m:.1f} m | Cal={LIDAR_DISTANCE_SCALE:.4f}x | Points={visible_points} | "
        f"Avoid={'ON' if AUTO_AVOIDANCE_ENABLED else 'OFF'}:{len(avoidance_route)}pts | FPS={fps:.1f}"
    )
    if front_obstacle_distance_mm is None:
        stop_status = f"Front stop: ON | limit < {FRONT_STOP_DISTANCE_MM} mm | cone ±{FRONT_CONE_HALF_ANGLE_DEG} deg | no front object"
        stop_color = TEXT_MUTED
    elif obstacle_detected:
        stop_status = f"STOP: front object {front_obstacle_distance_mm:.0f} mm | cone ±{FRONT_CONE_HALF_ANGLE_DEG} deg"
        stop_color = RED
    else:
        stop_status = f"Front stop: nearest front object {front_obstacle_distance_mm:.0f} mm | cone ±{FRONT_CONE_HALF_ANGLE_DEG} deg"
        stop_color = TEXT_MUTED

    help_text = "C clear map | S stop | R reset all | L calibrate | A avoid on/off | UP/DOWN zoom | [/] stop angle | ESC/Q quit"

    text_x = lidar_rect.x + pad
    max_text_w = lidar_rect.width - pad * 2
    y0 = footer_top + 8
    draw_text_fit(surface, pose_text, tiny_font, TEXT, text_x, y0, max_text_w)
    draw_text_fit(surface, info_text, tiny_font, TEXT_MUTED, text_x, y0 + 17, max_text_w)
    draw_text_fit(surface, stop_status, tiny_font, stop_color, text_x, y0 + 34, max_text_w)
    draw_text_fit(surface, help_text, tiny_font, TEXT_MUTED, text_x, y0 + 51, max_text_w)


def draw_serial_log(surface):
    available_bottom = status_rect.y - 10
    log_x = 25
    log_y = max(365, available_bottom - 96)

    if log_y + 22 >= available_bottom:
        return

    draw_text_fit(surface, "Arduino feedback", tiny_font, TEXT, log_x, log_y, 290)
    max_lines = max(1, min(5, (available_bottom - log_y - 22) // 16))

    for i, line in enumerate(list(serial_log)[-max_lines:]):
        yy = log_y + 20 + i * 16
        draw_text_fit(surface, line, tiny_font, TEXT_MUTED, log_x, yy, 290)

# =========================
# MAIN LOOP
# =========================
running = True

try:
    while running:
        read_arduino_lines()
        read_lidar_data()
        update_front_obstacle_stop()

        fps = clock.get_fps()
        screen.fill(BG)

        title = font.render("Robot Serial Controller", True, TEXT)
        screen.blit(title, (25, 25))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.VIDEORESIZE and not fullscreen_mode:
                new_w = max(MIN_WIDTH, event.w)
                new_h = max(MIN_HEIGHT, event.h)
                screen = pygame.display.set_mode((new_w, new_h), pygame.RESIZABLE)
                update_layout()

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key in (pygame.K_F11, pygame.K_f):
                    toggle_fullscreen()
                elif event.key == pygame.K_c:
                    clear_lidar_map()
                elif event.key == pygame.K_s:
                    send_stop_command("manual key")
                elif event.key == pygame.K_r:
                    reset_all()
                elif event.key == pygame.K_l:
                    calibrate_lidar_now()
                elif event.key == pygame.K_a:
                    AUTO_AVOIDANCE_ENABLED = not AUTO_AVOIDANCE_ENABLED
                    last_message = f"Auto avoidance {'ON' if AUTO_AVOIDANCE_ENABLED else 'OFF'}"
                elif event.key == pygame.K_UP:
                    max_range_m = max(1.0, max_range_m - 0.5)
                    last_message = f"LiDAR range set to {max_range_m:.1f} m"
                elif event.key == pygame.K_DOWN:
                    max_range_m = min(12.0, max_range_m + 0.5)
                    last_message = f"LiDAR range set to {max_range_m:.1f} m"
                elif event.key == pygame.K_LEFTBRACKET:
                    FRONT_CONE_HALF_ANGLE_DEG = max(5, FRONT_CONE_HALF_ANGLE_DEG - 5)
                    last_message = f"Front emergency angle decreased to ±{FRONT_CONE_HALF_ANGLE_DEG} deg"
                elif event.key == pygame.K_RIGHTBRACKET:
                    FRONT_CONE_HALF_ANGLE_DEG = min(90, FRONT_CONE_HALF_ANGLE_DEG + 5)
                    last_message = f"Front emergency angle increased to ±{FRONT_CONE_HALF_ANGLE_DEG} deg"

            for box in input_boxes:
                box.handle_event(event)

            if event.type == pygame.MOUSEBUTTONDOWN:
                for button in buttons:
                    if button.is_clicked(event.pos):
                        button.action()

        update_mouse_cursor()

        for box in input_boxes:
            box.draw(screen)

        for button in buttons:
            button.draw(screen)

        draw_waypoint_list(screen)
        draw_lidar_screen(screen, fps)

        # Bottom status area, automatically stretched for full screen or resized window.
        pygame.draw.rect(screen, PANEL_BG, status_rect)
        pygame.draw.rect(screen, BORDER, status_rect, 1)

        draw_text_fit(screen, last_message, small_font, GREEN, status_rect.x + 15, status_rect.y + 8, status_rect.width - 30)
        draw_text_fit(
            screen,
            "Arduino command: (X_mm,Y_mm,wait_seconds):  |  Feedback: X>Y>Heading",
            tiny_font,
            TEXT_MUTED,
            status_rect.x + 15,
            status_rect.y + 34,
            status_rect.width - 30,
        )
        draw_serial_log(screen)

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
