"""
Blue Bin Motor Control Integration Script - Raspberry Pi

Requirements (install on RPi before running):
    sudo apt-get install python3-opencv python3-numpy python3-serial
    # OR via pip:
    pip3 install opencv-python opencv-contrib-python numpy pyserial

This script:
 - Detects blue bin using USB webcam and color detection
 - Centers the bin in the camera view by sending motor commands via serial
 - Avoids obstacles using triple ultrasonic sensors
 - Movement logic:
   * If bin is left of center → turn left
   * If bin is right of center → turn right
   * If bin is relatively centered → move forward
   * No bin detected → stop motors (fallback to manual control)
   * Obstacle detected → avoid and navigate around it
 - Uses the same motor power values and serial device as aruco_motor_control.py
 - Can run headless (no display) for production use

Control Strategy:
 - CENTER_TOLERANCE: pixel range for "centered" (default ±50 pixels)
 - FORWARD_THRESHOLD: minimum bin size to move forward (prevents moving when far)
 - Motor commands sent via serial to Arduino/motor controller at /dev/ttyACM2
 - Obstacle sensors on /dev/ttyACM0

Obstacle Avoidance:
 - Front < 10cm: STOP and navigate around
 - Front < 30cm: SLOW DOWN
 - Side < 15cm: Bias away from that side
 - All blocked: BACKUP and turn

Author: Generated for MakeCU project
"""

import sys
import logging
import argparse
import time
import serial
from blue_bin_detector import BlueBinDetector
from triple_ultrasonic_sensor import TripleUltrasonicSensor


# Motor configuration (from aruco_motor_control.py)
FORWARD_LEFT_POWER = 260
FORWARD_RIGHT_POWER = -260
LEFT_L_POWER = 260
LEFT_R_POWER = 260
BACKUP_POWER = -200  # Power for backing up

# Serial configuration - PORT 1 FOR MOTORS, PORT 0 FOR SENSORS
MOTOR_SERIAL_PORT = '/dev/ttyACM2'
SENSOR_SERIAL_PORT = '/dev/ttyACM0'
MOTOR_BAUD_RATE = 57600
SENSOR_BAUD_RATE = 9600
SERIAL_TIMEOUT = 1

# Detection and control parameters
CENTER_TOLERANCE = 50  # pixels - if bin is within ±50px of center, consider it centered
TURN_SPEED_DIVISOR = 2  # reduce turn speed for smoother centering
MIN_BIN_DISTANCE = 0.15  # minimum distance estimate (0-1) to consider bin close enough

# Obstacle avoidance thresholds (cm)
DANGER_DISTANCE = 10.0    # Stop and navigate around
SLOW_DISTANCE = 30.0      # Slow down
SIDE_WARNING = 15.0       # Bias away from side obstacles
BACKUP_DURATION = 1.0     # Seconds to backup when all blocked
TURN_DURATION = 0.5       # Seconds to turn after backup


def send_motor_command(ser, left_power, right_power):
    """
    Send motor command to Arduino/motor controller via serial.
    Format: "left,right>" (e.g., "260,-260>")
    """
    command = f"{left_power},{right_power}>"
    try:
        ser.write(command.encode())
        logging.debug(f"Motor command sent: {command.strip()}")
    except serial.SerialException as e:
        logging.error(f"Failed to send motor command: {e}")


def stop_motors(ser):
    """Stop both motors."""
    send_motor_command(ser, 0, 0)


def backup_and_turn(ser):
    """
    Backup and turn when all sensors are blocked.
    """
    logging.info("All sensors blocked - backing up and turning...")
    
    # Backup for 1 second
    send_motor_command(ser, BACKUP_POWER, BACKUP_POWER)
    time.sleep(BACKUP_DURATION)
    
    # Turn right by default
    turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
    turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
    send_motor_command(ser, turn_left, turn_right)
    time.sleep(TURN_DURATION)
    
    # Stop
    stop_motors(ser)
    logging.info("Backup and turn complete")


def compute_motor_command_with_obstacles(angle, distance_estimate, frame_width, obstacles):
    """
    Compute motor commands based on bin angle, distance, and obstacle detection.
    
    Args:
        angle: horizontal angle to bin (-90 to +90 degrees, negative=left, positive=right)
        distance_estimate: bin size estimate (0-1, higher = closer)
        frame_width: width of the camera frame (for reference)
        obstacles: dict with 'front', 'left', 'right' distances in cm
    
    Returns:
        tuple: (left_power, right_power, action_description, should_backup)
    """
    front = obstacles['front']
    left = obstacles['left']
    right = obstacles['right']
    
    # Check if all sensors are blocked
    if front < DANGER_DISTANCE and left < SIDE_WARNING and right < SIDE_WARNING:
        return (0, 0, "All blocked - need backup", True)
    
    # Check if front is blocked
    if front < DANGER_DISTANCE:
        # Need to navigate around - turn towards clearer side
        if left < right:
            # Right side is clearer - turn right
            turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
            turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
            return (turn_left, turn_right, f"Front blocked ({front:.0f}cm) - turn right", False)
        else:
            # Left side is clearer - turn left
            turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
            turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
            return (-turn_left, -turn_right, f"Front blocked ({front:.0f}cm) - turn left", False)
    
    # Convert angle to approximate pixel offset for bin tracking
    frame_center = frame_width / 2.0
    pixel_offset_estimate = (angle / 30.0) * frame_center
    
    # Determine speed based on front distance
    if front < SLOW_DISTANCE:
        # Slow zone
        forward_left = FORWARD_LEFT_POWER // 2
        forward_right = FORWARD_RIGHT_POWER // 2
        speed_label = "slow"
    else:
        # Normal speed
        forward_left = FORWARD_LEFT_POWER
        forward_right = FORWARD_RIGHT_POWER
        speed_label = "normal"
    
    # Adjust for side obstacles
    side_bias = ""
    if left < SIDE_WARNING:
        # Obstacle on left - bias right
        side_bias = " (avoiding left)"
        # Reduce left motor power to steer right
        forward_left = int(forward_left * 0.7)
    elif right < SIDE_WARNING:
        # Obstacle on right - bias left
        side_bias = " (avoiding right)"
        # Reduce right motor power to steer left
        forward_right = int(forward_right * 0.7)
    
    # Check if bin is centered horizontally
    if abs(pixel_offset_estimate) <= CENTER_TOLERANCE:
        # Bin is centered - move forward if bin is close enough
        if distance_estimate >= MIN_BIN_DISTANCE:
            return (forward_left, forward_right, f"Forward ({speed_label}){side_bias}", False)
        else:
            # Bin too small/far - just stop
            return (0, 0, "Stop (bin too far)", False)
    
    # Bin is off-center - turn to center it
    elif angle < 0:
        # Bin is to the left of center - turn left
        turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
        turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
        return (-turn_left, -turn_right, f"Turn left{side_bias}", False)
    else:
        # Bin is to the right of center - turn right
        turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
        turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
        return (turn_left, turn_right, f"Turn right{side_bias}", False)


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Blue Bin Motor Control for Raspberry Pi with Obstacle Avoidance')
    parser.add_argument('--headless', action='store_true', 
                        help='Run without display window (for SSH/headless operation)')
    parser.add_argument('--camera', type=int, default=0, 
                        help='Camera index (default: 0)')
    parser.add_argument('--motor-serial', type=str, default=MOTOR_SERIAL_PORT,
                        help=f'Motor serial port (default: {MOTOR_SERIAL_PORT})')
    parser.add_argument('--sensor-serial', type=str, default=SENSOR_SERIAL_PORT,
                        help=f'Sensor serial port (default: {SENSOR_SERIAL_PORT})')
    parser.add_argument('--motor-baud', type=int, default=MOTOR_BAUD_RATE,
                        help=f'Motor baud rate (default: {MOTOR_BAUD_RATE})')
    parser.add_argument('--sensor-baud', type=int, default=SENSOR_BAUD_RATE,
                        help=f'Sensor baud rate (default: {SENSOR_BAUD_RATE})')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging and camera visualization')
    args = parser.parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")

    # 1) Initialize serial connection to motor controller
    try:
        motor_ser = serial.Serial(
            args.motor_serial, 
            baudrate=args.motor_baud, 
            timeout=SERIAL_TIMEOUT,
            write_timeout=1.0  # Add write timeout to prevent blocking
        )
        time.sleep(2)  # Wait for device initialization
        logging.info(f"Motor serial connection established: {args.motor_serial} @ {args.motor_baud} baud")
        
        # Try to stop motors, but don't fail if it doesn't work yet
        try:
            stop_motors(motor_ser)
            logging.info("Motors initialized to stop position")
        except Exception as e:
            logging.warning(f"Could not send initial stop command: {e}")
            logging.warning("Will retry on first loop iteration")
            
    except serial.SerialException as e:
        logging.error(f"Failed to open motor serial port {args.motor_serial}: {e}")
        sys.exit(1)

    # 2) Initialize ultrasonic sensors
    try:
        sensors = TripleUltrasonicSensor(
            port=args.sensor_serial,
            baudrate=args.sensor_baud,
            debug=args.debug
        )
        logging.info(f"Ultrasonic sensors initialized: {args.sensor_serial} @ {args.sensor_baud} baud")
    except Exception as e:
        logging.error(f"Failed to initialize ultrasonic sensors: {e}")
        stop_motors(motor_ser)
        motor_ser.close()
        sys.exit(1)

    # 3) Initialize Blue Bin Detector
    try:
        # Only show debug windows if not headless AND debug is enabled
        show_debug = args.debug and not args.headless
        detector = BlueBinDetector(camera_index=args.camera, debug=show_debug)
        
        # Get frame dimensions from config
        import config_vision as config
        frame_width = config.CAMERA_WIDTH
        frame_height = config.CAMERA_HEIGHT
        
        logging.info(f"Blue Bin Detector initialized: {frame_width}x{frame_height}")
        logging.info(f"Headless mode: {args.headless}")
        logging.info(f"Obstacle thresholds: Danger={DANGER_DISTANCE}cm, Slow={SLOW_DISTANCE}cm, Side={SIDE_WARNING}cm")
        if not args.headless:
            logging.info("Press Ctrl+C to quit.")
        
    except Exception as e:
        logging.error(f"Failed to initialize Blue Bin Detector: {e}")
        stop_motors(motor_ser)
        motor_ser.close()
        sensors.close()
        sys.exit(1)

    last_action = "Stop (no bin)"
    frame_count = 0
    detection_count = 0
    no_detection_count = 0
    
    try:
        while True:
            frame_count += 1
            
            # Read obstacle sensors
            obstacles = sensors.read_sensors()
            
            # Detect blue bin
            detection = detector.detect_blue_bin()
            
            # Debug logging for detection
            if args.debug:
                if detection is not None:
                    angle, dist_est, conf = detection
                    logging.debug(f"[DETECTION] Angle:{angle:+.1f}° Dist:{dist_est:.2f} Conf:{conf:.2f}")
                else:
                    logging.debug(f"[NO DETECTION] Frame {frame_count}")

            # Process detection results
            if detection is not None:
                detection_count += 1
                angle, distance_estimate, confidence = detection
                
                # Log first detection
                if last_action == "Stop (no bin)":
                    logging.info(f"✓ Blue bin detected! Angle:{angle:+.1f}° Dist:{distance_estimate:.2f} Conf:{confidence:.2f}")
                
                # Compute motor command based on bin position and obstacles
                left_power, right_power, action, should_backup = compute_motor_command_with_obstacles(
                    angle, distance_estimate, frame_width, obstacles
                )
                
                # Check if we need to backup and turn
                if should_backup:
                    backup_and_turn(motor_ser)
                    last_action = "Backed up and turned"
                else:
                    send_motor_command(motor_ser, left_power, right_power)
                    
                    # Log info periodically or when action changes
                    if action != last_action or frame_count % 30 == 0:
                        logging.info(
                            f"Bin: {angle:+.1f}° | Dist: {distance_estimate:.2f} | "
                            f"Obstacles: F={obstacles['front']:.0f} L={obstacles['left']:.0f} R={obstacles['right']:.0f}cm | "
                            f"Action: {action}"
                        )
                        last_action = action

            else:
                no_detection_count += 1
                # No bin detected - stop motors
                stop_motors(motor_ser)
                if last_action != "Stop (no bin)":
                    logging.info(f"✗ Blue bin lost after {detection_count} detections")
                    last_action = "Stop (no bin)"
                
                # Periodic logging when no bin detected
                if frame_count % 50 == 0:
                    logging.info(f"No bin detected for {no_detection_count} frames (total frames: {frame_count})")

            # Small delay to prevent overwhelming the system
            time.sleep(0.05)

    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received — exiting.")
    finally:
        # Cleanup
        stop_motors(motor_ser)
        motor_ser.close()
        sensors.close()
        detector.cleanup()
        
        # Print statistics
        detector_stats = detector.get_statistics()
        sensor_stats = sensors.get_statistics()
        
        logging.info("\n" + "="*60)
        logging.info("STATISTICS")
        logging.info("="*60)
        logging.info(f"Camera frames processed: {detector_stats['total_frames']}")
        logging.info(f"Bin detections: {detector_stats['detected_frames']} ({detector_stats['detection_rate']:.1f}%)")
        logging.info(f"Sensor readings: {sensor_stats['total_readings']}")
        logging.info(f"Valid sensor readings: {sensor_stats['valid_readings']} ({sensor_stats['success_rate']:.1f}%)")
        logging.info("="*60)
        logging.info("Motors stopped, serial closed, sensors/camera released. Goodbye.")


if __name__ == "__main__":
    main()
