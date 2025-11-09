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

# Serial configuration - PORT 1 FOR MOTORS, PORT 0 FOR SENSORS, PORT 1 FOR SERVOS
MOTOR_SERIAL_PORT = '/dev/ttyACM2'
SENSOR_SERIAL_PORT = '/dev/ttyACM0'
SERVO_SERIAL_PORT = '/dev/ttyACM1'
MOTOR_BAUD_RATE = 57600
SENSOR_BAUD_RATE = 9600
SERVO_BAUD_RATE = 9600
SERIAL_TIMEOUT = 1

# Detection and control parameters
CENTER_TOLERANCE = 50  # pixels - if bin is within ±50px of center, consider it centered
TURN_SPEED_DIVISOR = 4  # reduce turn speed for smoother centering (higher = gentler turns)
MIN_BIN_DISTANCE = 0.15  # minimum distance estimate (0-1) to consider bin close enough

# Obstacle avoidance thresholds (cm)
DANGER_DISTANCE = 10.0    # Stop and navigate around
SLOW_DISTANCE = 30.0      # Slow down
SIDE_WARNING = 15.0       # Bias away from side obstacles
BACKUP_DURATION = 1.0     # Seconds to backup when all blocked
TURN_DURATION = 0.5       # Seconds to turn after backup

# Blue bin arrival detection (using camera, not ultrasonic)
BLUE_FULL_FRAME_THRESHOLD = 0.90  # 90% of frame blue = reached bin
CENTER_ANGLE_TOLERANCE = 10.0     # degrees - bin must be centered
STOP_MIN_CONFIDENCE = 0.3         # minimum confidence for stopping

# Servo arm control
SERVO_EXTEND_COMMAND = 'e'
SERVO_RETRACT_COMMAND = 'r'
SERVO_CYCLE_COUNT = 3             # Number of extend/retract cycles
SERVO_ACTION_DELAY = 2.0          # Seconds to wait for servo to complete action


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


def hardcoded_obstacle_avoidance(ser):
    """
    Hardcoded obstacle avoidance path for demo:
    1. Turn left for TURN_DURATION
    2. Pause 0.5s
    3. Move straight for 2s
    4. Pause 0.5s
    5. Turn right for TURN_DURATION (same duration as left turn)
    
    Args:
        ser: Motor serial connection
    """
    logging.info("🔄 Starting hardcoded obstacle avoidance maneuver")
    
    # Use FULL turn power for obstacle avoidance (not divided for centering)
    turn_left = LEFT_L_POWER
    turn_right = LEFT_R_POWER
    
    # Step 1: Turn left with FULL power
    logging.info("  Step 1: Turning left...")
    send_motor_command(ser, -turn_left, -turn_right)
    time.sleep(TURN_DURATION)
    stop_motors(ser)
    
    # Step 2: Pause 0.5s
    logging.info("  Step 2: Pause...")
    time.sleep(0.5)
    
    # Step 3: Move straight for 2s with FULL power
    logging.info("  Step 3: Moving forward...")
    send_motor_command(ser, FORWARD_LEFT_POWER, FORWARD_RIGHT_POWER)
    time.sleep(2.0)
    stop_motors(ser)
    
    # Step 4: Pause 0.5s
    logging.info("  Step 4: Pause...")
    time.sleep(0.5)
    
    # Step 5: Turn right with FULL power (same duration as left turn)
    logging.info("  Step 5: Turning right...")
    send_motor_command(ser, turn_left, turn_right)
    time.sleep(TURN_DURATION)
    stop_motors(ser)
    
    logging.info("✓ Hardcoded obstacle avoidance complete")
    return True


def perform_servo_cycles(motor_ser, servo_ser):
    """
    Perform extend/retract cycles with servo arms after reaching blue bin.
    Robot must stay completely stopped during this operation.
    
    Args:
        motor_ser: Motor serial connection (to keep robot stopped)
        servo_ser: Servo serial connection (to control arms)
    """
    logging.info("🤖 Starting servo arm cycles")
    
    # Ensure motors are stopped
    stop_motors(motor_ser)
    
    for cycle in range(1, SERVO_CYCLE_COUNT + 1):
        logging.info(f"  Cycle {cycle}/{SERVO_CYCLE_COUNT}")
        
        # Extend arms
        logging.info(f"    → Extending arms...")
        try:
            servo_ser.write(SERVO_EXTEND_COMMAND.encode())
            servo_ser.flush()
        except serial.SerialException as e:
            logging.error(f"Failed to send extend command: {e}")
        
        time.sleep(SERVO_ACTION_DELAY)
        
        # Keep motors stopped during servo operation
        stop_motors(motor_ser)
        
        # Retract arms
        logging.info(f"    ← Retracting arms...")
        try:
            servo_ser.write(SERVO_RETRACT_COMMAND.encode())
            servo_ser.flush()
        except serial.SerialException as e:
            logging.error(f"Failed to send retract command: {e}")
        
        time.sleep(SERVO_ACTION_DELAY)
        
        # Keep motors stopped during servo operation
        stop_motors(motor_ser)
    
    logging.info("✓ Servo cycles complete")
    logging.info("🎉 FINISHED - Mission complete!")


def search_360_for_bin(ser, detector):
    """
    Rotate 360 degrees looking for the blue bin.
    Stop when bin is found or after full rotation.
    
    Args:
        ser: Motor serial connection
        detector: BlueBinDetector instance
        
    Returns:
        True if bin found, False if not
    """
    logging.info("🔍 Starting 360° search for blue bin")
    
    # Rotation parameters
    turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
    turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
    
    # Rotate in small steps (36 steps = 10° each)
    steps = 36
    step_duration = 0.3  # seconds per step
    
    for step in range(steps):
        # Turn right slowly
        send_motor_command(ser, turn_left // 2, turn_right // 2)
        time.sleep(step_duration)
        
        # Stop and check for bin
        stop_motors(ser)
        time.sleep(0.1)
        
        detection = detector.detect_blue_bin()
        if detection is not None:
            angle, distance_estimate, confidence = detection
            logging.info(f"✓ Bin found at step {step+1}/{steps}! Angle:{angle:+.1f}° Dist:{distance_estimate:.2f}")
            return True
        
        # Brief pause before next step
        time.sleep(0.1)
    
    # Completed full rotation without finding bin
    logging.info("✗ 360° search complete - no bin found")
    stop_motors(ser)
    return False


def compute_motor_command_with_obstacles(angle, distance_estimate, confidence, blue_ratio, frame_width, obstacles):
    """
    Compute motor commands based on bin angle, distance, blue coverage, and obstacle detection.
    Returns: (left_power, right_power, action_description, trigger_value)
    
    trigger_value:
        - False: Normal movement
        - True: All blocked - need backup_and_turn
        - 'left'/'right': Front blocked - use hardcoded_obstacle_avoidance
    """
    front = obstacles['front']
    left = obstacles['left']
    right = obstacles['right']

    # ✅ 1) ARRIVAL: Stop when frame is mostly blue (90%) and bin is centered
    # Uses CAMERA to detect arrival, not ultrasonic sensor
    if (blue_ratio >= BLUE_FULL_FRAME_THRESHOLD and
        abs(angle) <= CENTER_ANGLE_TOLERANCE and
        confidence >= STOP_MIN_CONFIDENCE):
        return (0, 0, f"🎯 REACHED BLUE BIN (blue={blue_ratio*100:.0f}%) - STOP", False)

    # ✅ 2) All sensors blocked -> only treat as "blocked" if we're NOT aligned with the bin
    if (front < DANGER_DISTANCE and
        left < SIDE_WARNING and
        right < SIDE_WARNING and
        abs(angle) > CENTER_ANGLE_TOLERANCE):
        return (0, 0, "All blocked - need backup", True)

    # ✅ 3) Front blocked -> trigger hardcoded avoidance if obstacle is NOT the bin we're aligned with
    if front < DANGER_DISTANCE and abs(angle) > CENTER_ANGLE_TOLERANCE:
        # Need to navigate around - determine clearer side
        clearer_side = 'right' if right > left else 'left'
        return (0, 0, f"Front blocked ({front:.0f}cm) - avoid {clearer_side}", clearer_side)

    # (unchanged below - normal tracking)

    # Convert angle to approximate pixel offset for bin tracking
    frame_center = frame_width / 2.0
    pixel_offset_estimate = (angle / 30.0) * frame_center

    # Determine speed based on front distance
    if front < SLOW_DISTANCE:
        forward_left = FORWARD_LEFT_POWER // 2
        forward_right = FORWARD_RIGHT_POWER // 2
        speed_label = "slow"
    else:
        forward_left = FORWARD_LEFT_POWER
        forward_right = FORWARD_RIGHT_POWER
        speed_label = "normal"

    # Adjust for side obstacles
    side_bias = ""
    if left < SIDE_WARNING:
        side_bias = " (avoiding left)"
        forward_left = int(forward_left * 0.7)
    elif right < SIDE_WARNING:
        side_bias = " (avoiding right)"
        forward_right = int(forward_right * 0.7)

    # Bin centered → go forward (distance estimation unreliable, rely on front sensor instead)
    if abs(pixel_offset_estimate) <= CENTER_TOLERANCE:
        return (forward_left, forward_right, f"Forward ({speed_label}){side_bias}", False)

    # Bin left/right → turn to re-center
    if angle < 0:
        turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
        turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
        return (-turn_left, -turn_right, f"Turn left{side_bias}", False)
    else:
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
    parser.add_argument('--servo-serial', type=str, default=SERVO_SERIAL_PORT,
                        help=f'Servo serial port (default: {SERVO_SERIAL_PORT})')
    parser.add_argument('--motor-baud', type=int, default=MOTOR_BAUD_RATE,
                        help=f'Motor baud rate (default: {MOTOR_BAUD_RATE})')
    parser.add_argument('--sensor-baud', type=int, default=SENSOR_BAUD_RATE,
                        help=f'Sensor baud rate (default: {SENSOR_BAUD_RATE})')
    parser.add_argument('--servo-baud', type=int, default=SERVO_BAUD_RATE,
                        help=f'Servo baud rate (default: {SERVO_BAUD_RATE})')
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

    # 2) Initialize servo controller
    try:
        servo_ser = serial.Serial(
            args.servo_serial,
            baudrate=args.servo_baud,
            timeout=SERIAL_TIMEOUT,
            write_timeout=1.0
        )
        time.sleep(2)  # Wait for device initialization
        logging.info(f"Servo serial connection established: {args.servo_serial} @ {args.servo_baud} baud")
    except serial.SerialException as e:
        logging.error(f"Failed to open servo serial port {args.servo_serial}: {e}")
        stop_motors(motor_ser)
        motor_ser.close()
        sys.exit(1)

    # 3) Initialize ultrasonic sensors
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
        servo_ser.close()
        sys.exit(1)

    # 4) Initialize Blue Bin Detector
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
        servo_ser.close()
        sensors.close()
        sys.exit(1)


    last_action = "Stop (no bin)"
    frame_count = 0
    detection_count = 0
    no_detection_count = 0
    consecutive_no_detection = 0  # Track consecutive frames without detection
    in_obstacle_avoidance = False  # Flag to pause bin tracking during arc-around
    mission_complete = False  # Flag to end program after servo cycles
    no_detection_count = 0
    consecutive_no_detection = 0  # Track consecutive frames without detection
    in_obstacle_avoidance = False  # Flag to pause bin tracking during arc-around
    
    try:
        while True:
            frame_count += 1
            
            # Read obstacle sensors
            obstacles = sensors.read_sensors()
            
            # Detect blue bin (only if not in obstacle avoidance mode)
            if not in_obstacle_avoidance:
                detection = detector.detect_blue_bin()
            else:
                detection = None  # Ignore camera during obstacle maneuver
            
            # Debug logging for detection
            if args.debug:
                if detection is not None:
                    angle, dist_est, conf, blue_r = detection
                    logging.debug(f"[DETECTION] Angle:{angle:+.1f}° Dist:{dist_est:.2f} Conf:{conf:.2f} Blue:{blue_r*100:.1f}%")
                else:
                    logging.debug(f"[NO DETECTION] Frame {frame_count}")

            # Process detection results
            if detection is not None:
                detection_count += 1
                consecutive_no_detection = 0  # Reset counter
                angle, distance_estimate, confidence, blue_ratio = detection
                
                # Log first detection
                if last_action == "Stop (no bin)" or last_action.startswith("Searching"):
                    logging.info(f"✓ Blue bin detected! Angle:{angle:+.1f}° Dist:{distance_estimate:.2f} Conf:{confidence:.2f} Blue:{blue_ratio*100:.1f}%")
                
                # Compute motor command based on bin position and obstacles
                result = compute_motor_command_with_obstacles(
                    angle, distance_estimate, confidence, blue_ratio, frame_width, obstacles
                )
                
                # Unpack result - can be 4 values or special case with clearer_side
                if len(result) == 4:
                    left_power, right_power, action, trigger_value = result
                    
                    # Check if trigger_value is True (backup) or a string (hardcoded path)
                    if trigger_value is True:
                        # Old backup behavior for all-blocked scenario
                        backup_and_turn(motor_ser)
                        last_action = "Backed up and turned"
                    elif isinstance(trigger_value, str):
                        # Front blocked - use hardcoded obstacle avoidance path
                        # Retry up to 3 times before giving up
                        in_obstacle_avoidance = True
                        logging.info(f"🚧 Obstacle detected - pausing bin tracking for hardcoded avoidance")
                        
                        retry_count = 0
                        max_retries = 3
                        path_clear = False
                        
                        while retry_count < max_retries:
                            hardcoded_obstacle_avoidance(motor_ser)
                            retry_count += 1
                            
                            # Check if front is still blocked after maneuver
                            time.sleep(0.2)  # Brief delay to get fresh sensor reading
                            obstacles_check = sensors.read_sensors()
                            front_dist = obstacles_check.get('front', 0.0) if obstacles_check else 0.0
                            front_dist = front_dist if front_dist is not None else 0.0
                            
                            if front_dist > DANGER_DISTANCE:
                                logging.info(f"✓ Path clear after {retry_count} attempt(s)")
                                path_clear = True
                                break
                            else:
                                logging.info(f"⚠ Front still blocked ({front_dist:.0f}cm) - retry {retry_count}/{max_retries}")
                        
                        # If still blocked after retries, fall back to backup_and_turn
                        if not path_clear:
                            logging.info(f"❌ Path still blocked after {max_retries} attempts - using backup and turn")
                            backup_and_turn(motor_ser)
                        
                        in_obstacle_avoidance = False
                        logging.info(f"✓ Obstacle avoidance complete - resuming bin tracking")
                        last_action = f"Hardcoded obstacle avoidance"
                    else:
                        # Normal movement
                        send_motor_command(motor_ser, left_power, right_power)
                        
                        # Check if we reached the blue bin (action contains "REACHED BLUE BIN")
                        if "REACHED BLUE BIN" in action:
                            logging.info("=" * 60)
                            logging.info("🎯 BLUE BIN REACHED!")
                            logging.info("=" * 60)
                            
                            # Ensure motors are stopped
                            stop_motors(motor_ser)
                            
                            # Perform servo cycles
                            perform_servo_cycles(motor_ser, servo_ser)
                            
                            # Set flag to end program
                            mission_complete = True
                            break  # Exit main loop
                        
                        # Log info periodically or when action changes
                        if action != last_action or frame_count % 30 == 0:
                            logging.info(
                                f"Bin: {angle:+.1f}° | Blue: {blue_ratio*100:.0f}% | "
                                f"Obstacles: F={obstacles['front']:.0f} L={obstacles['left']:.0f} R={obstacles['right']:.0f}cm | "
                                f"Action: {action}"
                            )
                            last_action = action

            else:
                no_detection_count += 1
                consecutive_no_detection += 1
                
                # Check if we should trigger 360° search (after 10 consecutive frames without detection)
                if consecutive_no_detection >= 10 and not in_obstacle_avoidance:
                    logging.info(f"⚠ Blue bin lost for {consecutive_no_detection} frames - initiating 360° search")
                    last_action = "Searching 360°"
                    
                    # Perform 360° search
                    bin_found = search_360_for_bin(motor_ser, detector)
                    
                    if bin_found:
                        consecutive_no_detection = 0  # Reset counter
                        logging.info("✓ 360° search successful - resuming tracking")
                        last_action = "Resumed after search"
                    else:
                        logging.info("✗ 360° search failed - stopping and waiting")
                        stop_motors(motor_ser)
                        last_action = "Stop (no bin after search)"
                        consecutive_no_detection = 0  # Reset to avoid repeated searches
                else:
                    # No bin detected - stop motors
                    stop_motors(motor_ser)
                    if last_action != "Stop (no bin)" and consecutive_no_detection == 1:
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
        servo_ser.close()
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
        
        if mission_complete:
            logging.info("✓ Mission completed successfully!")
        
        logging.info("Motors stopped, serial closed, sensors/camera released. Goodbye.")
        logging.info("Motors stopped, serial closed, sensors/camera released. Goodbye.")


if __name__ == "__main__":
    main()