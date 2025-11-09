"""
Test ArUco Detection and Motor Control - Standalone Script

This script ONLY does ArUco marker detection and movement.
Use this to test the ArUco tracking without the blue bin phase.

Usage:
    python3 test_aruco_only.py --debug
    python3 test_aruco_only.py --headless  # for RPi without display

Requirements:
    - ArUco marker ID 22 visible to camera
    - Motor controller on /dev/ttyACM2
    - Ultrasonic sensors on /dev/ttyACM0
"""

import sys
import logging
import argparse
import time
import serial
import cv2
import numpy as np
from triple_ultrasonic_sensor import TripleUltrasonicSensor


# Motor configuration
FORWARD_LEFT_POWER = 260
FORWARD_RIGHT_POWER = -260
LEFT_L_POWER = 260
LEFT_R_POWER = 260
BACKUP_POWER = -200

# Serial configuration
MOTOR_SERIAL_PORT = '/dev/ttyACM2'
SENSOR_SERIAL_PORT = '/dev/ttyACM0'
MOTOR_BAUD_RATE = 57600
SENSOR_BAUD_RATE = 9600
SERIAL_TIMEOUT = 1

# ArUco detection parameters
ARUCO_TARGET_ID = 22
ARUCO_FULL_FRAME_THRESHOLD = 0.70  # 70% of frame = reached
ARUCO_CENTER_TOLERANCE = 50        # pixels
ARUCO_MIN_AREA = 1000              # minimum area to move forward
TURN_SPEED_DIVISOR = 4             # gentler turns

# Obstacle thresholds
DANGER_DISTANCE = 10.0
SLOW_DISTANCE = 30.0
SIDE_WARNING = 15.0


def send_motor_command(ser, left_power, right_power):
    """Send motor command via serial."""
    command = f"{left_power},{right_power}>"
    try:
        ser.write(command.encode())
        logging.debug(f"Motor: {command.strip()}")
    except serial.SerialException as e:
        logging.error(f"Motor command failed: {e}")


def stop_motors(ser):
    """Stop both motors."""
    send_motor_command(ser, 0, 0)


def detect_aruco_marker(cap, aruco_dict, parameters, target_id=ARUCO_TARGET_ID):
    """
    Detect ArUco marker in current camera frame.
    Returns: (cx, cy, area, marker_id) or None
    """
    ret, frame = cap.read()
    if not ret or frame is None:
        return None, None
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if corners is None or len(corners) == 0 or ids is None:
        return None, frame
    
    # Look for target ID
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id == target_id:
            first = np.array(corners[i])
            pts = first.reshape((-1, 2))
            center = pts.mean(axis=0)
            cx, cy = center
            area = cv2.contourArea(pts)
            
            # Draw marker on frame
            cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[marker_id]]))
            cv2.circle(frame, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            
            return (cx, cy, area, marker_id), frame
    
    return None, frame


def compute_aruco_motor_command(cx, frame_width, marker_area, obstacles):
    """
    Compute motor commands to approach ArUco marker.
    Returns: (left_power, right_power, action_description, reached)
    """
    frame_center_x = frame_width / 2.0
    offset_x = cx - frame_center_x
    
    # Calculate area ratio
    frame_area = frame_width * 480
    area_ratio = marker_area / frame_area
    
    # Check if reached ArUco
    if area_ratio >= ARUCO_FULL_FRAME_THRESHOLD and abs(offset_x) <= ARUCO_CENTER_TOLERANCE:
        return (0, 0, f"🎯 REACHED ARUCO (area={area_ratio*100:.0f}%)", True)
    
    # Check obstacles
    front = obstacles.get('front', 200.0)
    left = obstacles.get('left', 200.0)
    right = obstacles.get('right', 200.0)
    
    # Front obstacle
    if front < DANGER_DISTANCE:
        return (0, 0, f"⚠ Front blocked ({front:.0f}cm)", False)
    
    # Check if centered
    if abs(offset_x) <= ARUCO_CENTER_TOLERANCE:
        # Centered - move forward if large enough
        if marker_area >= ARUCO_MIN_AREA:
            # Adjust speed based on distance
            if front < SLOW_DISTANCE:
                forward_left = FORWARD_LEFT_POWER // 2
                forward_right = FORWARD_RIGHT_POWER // 2
                return (forward_left, forward_right, "Forward slow (ArUco centered)", False)
            else:
                return (FORWARD_LEFT_POWER, FORWARD_RIGHT_POWER, "Forward (ArUco centered)", False)
        else:
            return (0, 0, "Stop (ArUco too far)", False)
    
    # Off-center - turn to center
    turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
    turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
    
    if offset_x < 0:
        # Marker left of center - turn left
        return (-turn_left, -turn_right, f"Turn left (offset={offset_x:.0f}px)", False)
    else:
        # Marker right of center - turn right
        return (turn_left, turn_right, f"Turn right (offset={offset_x:.0f}px)", False)


def main():
    parser = argparse.ArgumentParser(description='Test ArUco Detection Only')
    parser.add_argument('--headless', action='store_true', help='Run without display')
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    args = parser.parse_args()
    
    # Setup logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")
    
    # Initialize motor controller
    try:
        motor_ser = serial.Serial(
            MOTOR_SERIAL_PORT,
            baudrate=MOTOR_BAUD_RATE,
            timeout=SERIAL_TIMEOUT,
            write_timeout=1.0
        )
        time.sleep(2)
        logging.info(f"Motor serial: {MOTOR_SERIAL_PORT} @ {MOTOR_BAUD_RATE}")
        stop_motors(motor_ser)
    except serial.SerialException as e:
        logging.error(f"Motor serial failed: {e}")
        sys.exit(1)
    
    # Initialize ultrasonic sensors
    try:
        sensors = TripleUltrasonicSensor(
            port=SENSOR_SERIAL_PORT,
            baudrate=SENSOR_BAUD_RATE,
            debug=args.debug
        )
        logging.info(f"Sensors: {SENSOR_SERIAL_PORT} @ {SENSOR_BAUD_RATE}")
    except Exception as e:
        logging.error(f"Sensor init failed: {e}")
        stop_motors(motor_ser)
        motor_ser.close()
        sys.exit(1)
    
    # Initialize camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        logging.error(f"Camera {args.camera} failed to open")
        stop_motors(motor_ser)
        motor_ser.close()
        sensors.close()
        sys.exit(1)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    logging.info(f"Camera: {frame_width}x{frame_height}")
    
    # Initialize ArUco detection
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    elif hasattr(cv2.aruco, 'Dictionary_get'):
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    else:
        raise RuntimeError("Unable to get ArUco dictionary")
    
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        aruco_parameters = cv2.aruco.DetectorParameters_create()
    elif hasattr(cv2.aruco, 'DetectorParameters'):
        aruco_parameters = cv2.aruco.DetectorParameters()
    else:
        raise RuntimeError("Unable to get ArUco parameters")
    
    logging.info(f"ArUco: Looking for marker ID {ARUCO_TARGET_ID}")
    logging.info("Press Ctrl+C to quit")
    logging.info("="*60)
    
    last_action = "Searching"
    frame_count = 0
    reached = False
    
    try:
        while not reached:
            frame_count += 1
            
            # Read sensors
            obstacles = sensors.read_sensors()
            
            # Detect ArUco
            detection, frame = detect_aruco_marker(cap, aruco_dict, aruco_parameters, ARUCO_TARGET_ID)
            
            if detection is not None:
                cx, cy, area, marker_id = detection
                
                # Compute motor command
                left_power, right_power, action, reached = compute_aruco_motor_command(
                    cx, frame_width, area, obstacles
                )
                
                # Send command
                send_motor_command(motor_ser, left_power, right_power)
                
                # Log
                if action != last_action or frame_count % 30 == 0:
                    area_pct = (area / (frame_width * frame_height)) * 100
                    logging.info(
                        f"ID {marker_id} | Pos:({int(cx)},{int(cy)}) | "
                        f"Area:{int(area)} ({area_pct:.1f}%) | "
                        f"Obstacles: F={obstacles['front']:.0f} L={obstacles['left']:.0f} R={obstacles['right']:.0f}cm | "
                        f"Action: {action}"
                    )
                    last_action = action
                
                # Display frame
                if not args.headless and frame is not None:
                    # Draw frame center
                    center = (frame_width // 2, frame_height // 2)
                    cv2.circle(frame, center, 4, (255, 0, 0), -1)
                    cv2.line(frame, (center[0]-20, center[1]), (center[0]+20, center[1]), (255,0,0), 2)
                    cv2.line(frame, (center[0], center[1]-20), (center[0], center[1]+20), (255,0,0), 2)
                    
                    # Display info
                    cv2.putText(frame, action, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Offset: {int(cx - frame_width/2)}px", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    cv2.imshow("ArUco Test", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                if reached:
                    logging.info("="*60)
                    logging.info("🎯 ARUCO REACHED!")
                    logging.info("="*60)
                    stop_motors(motor_ser)
                    break
            
            else:
                # No ArUco detected
                stop_motors(motor_ser)
                
                if last_action != "Searching":
                    logging.info("✗ ArUco marker lost")
                    last_action = "Searching"
                
                if frame_count % 50 == 0:
                    logging.info(f"Searching for ArUco ID {ARUCO_TARGET_ID}... (frame {frame_count})")
                
                # Display frame
                if not args.headless and frame is not None:
                    cv2.putText(frame, "Searching for ArUco...", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow("ArUco Test", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            
            time.sleep(0.05)
    
    except KeyboardInterrupt:
        logging.info("\nKeyboardInterrupt - exiting")
    
    finally:
        stop_motors(motor_ser)
        motor_ser.close()
        sensors.close()
        cap.release()
        cv2.destroyAllWindows()
        logging.info("Cleanup complete")


if __name__ == "__main__":
    main()
