import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/meng/mDetectRobot/turtlebot3_ws/install/arduino_bridge'
