"""
Configuration for Vision-Guided Navigation
Single ultrasonic sensor + Camera for blue bin detection
"""

# ============================================================
# SERIAL PORT CONFIGURATION
# ============================================================

SENSOR_PORT = "COM5"      # Ultrasonic sensor port
BAUDRATE = 9600


# ============================================================
# DISTANCE THRESHOLDS (cm)
# ============================================================

DANGER_DISTANCE = 30.0    # Stop and avoid
WARNING_DISTANCE = 50.0   # Slow down
SAFE_DISTANCE = 80.0      # Full speed


# ============================================================
# CAMERA CONFIGURATION
# ============================================================

CAMERA_INDEX = 0          # 0 = default camera, 1 = USB camera

# Blue bin color detection (HSV color space)
BLUE_LOWER_HSV = (90, 100, 100)   # Lower bound for blue
BLUE_UPPER_HSV = (130, 255, 255)  # Upper bound for blue

# Minimum area for valid detection (pixels²)
MIN_BIN_AREA = 500

# Camera resolution
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


# ============================================================
# NAVIGATION PARAMETERS
# ============================================================

# Search rotation speed
SEARCH_ROTATION_SPEED = 35    # Slow rotation for scanning

# Navigation speeds
FULL_SPEED = 60
SLOW_SPEED = 40
TURN_SPEED = 45
AVOID_SPEED = 50

# Angle tolerances (degrees)
ANGLE_TOLERANCE = 5.0      # How close to target angle is "aligned"
SEARCH_STEP_ANGLE = 10.0   # Degrees to rotate per search step

# Distance to target (when to stop)
TARGET_REACHED_DISTANCE = 30.0  # cm - stop when this close


# ============================================================
# OBSTACLE AVOIDANCE STRATEGY
# ============================================================

# Avoidance turn angle (degrees)
AVOID_TURN_ANGLE = 45.0

# How long to move sideways when avoiding (seconds)
AVOID_MOVE_DURATION = 1.5

# Maximum number of avoidance attempts before giving up
MAX_AVOID_ATTEMPTS = 3


# ============================================================
# TIMING
# ============================================================

SEARCH_ROTATION_DURATION = 0.3   # Time per search step
MIN_ACTION_DURATION = 0.2        # Minimum time between actions
CAMERA_READ_INTERVAL = 0.1       # Time between camera reads


# ============================================================
# GPIO PIN CONFIGURATION
# ============================================================

LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 27
LEFT_MOTOR_ENABLE = 22

RIGHT_MOTOR_FORWARD = 23
RIGHT_MOTOR_BACKWARD = 24
RIGHT_MOTOR_ENABLE = 25