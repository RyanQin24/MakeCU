"""
ArUco Motor Control Integration Script - Raspberry Pi

Requirements (install on RPi before running):
    sudo apt-get install python3-opencv python3-numpy python3-serial
    # OR via pip:
    pip3 install opencv-python opencv-contrib-python numpy pyserial

This script:
 - Detects ArUco markers using USB webcam
 - Centers the marker in the camera view by sending motor commands via serial
 - Movement logic:
   * If marker is left of center → turn left
   * If marker is right of center → turn right
   * If marker is relatively centered → move forward
   * No marker detected → stop motors (fallback to manual WASD control)
 - Uses the same motor power values and serial device as receiver_pi.py
 - Can run headless (no display) for production use

Control Strategy:
 - CENTER_TOLERANCE: pixel range for "centered" (default ±50 pixels)
 - FORWARD_THRESHOLD: minimum marker size to move forward (prevents moving when far)
 - Motor commands sent via serial to Arduino/motor controller at /dev/ttyACM0

Author: Generated for user
"""

import sys
import logging
import argparse
import time
import serial
import cv2
import numpy as np


# Motor configuration (from receiver_pi.py)
FORWARD_LEFT_POWER = 260
FORWARD_RIGHT_POWER = -260
LEFT_L_POWER = 260
LEFT_R_POWER = 260

# Serial configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 57600
SERIAL_TIMEOUT = 1

# Detection and control parameters
CENTER_TOLERANCE = 50  # pixels - if marker is within ±50px of center, consider it centered
TURN_SPEED_DIVISOR = 2  # reduce turn speed for smoother centering
MIN_MARKER_AREA = 1000  # minimum marker area (pixels^2) to consider it valid/close enough


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


def compute_motor_command(cx, frame_width, marker_area):
    """
    Compute motor commands based on marker center position.
    
    Args:
        cx: marker center x-coordinate
        frame_width: width of the camera frame
        marker_area: area of the detected marker (in pixels^2)
    
    Returns:
        tuple: (left_power, right_power, action_description)
    """
    frame_center_x = frame_width / 2.0
    offset_x = cx - frame_center_x
    
    # Check if marker is centered horizontally
    if abs(offset_x) <= CENTER_TOLERANCE:
        # Marker is centered - move forward if marker is large enough (close enough)
        if marker_area >= MIN_MARKER_AREA:
            return (FORWARD_LEFT_POWER, FORWARD_RIGHT_POWER, "Forward (centered)")
        else:
            # Marker too small/far - just stop
            return (0, 0, "Stop (marker too far)")
    
    # Marker is off-center - turn to center it
    elif offset_x < 0:
        # Marker is to the left of center - turn left
        turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
        turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
        return (turn_left, turn_right, "Turn Left")
    else:
        # Marker is to the right of center - turn right
        turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
        turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
        return (-turn_left, -turn_right, "Turn Right")


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='ArUco Motor Control for Raspberry Pi')
    parser.add_argument('--headless', action='store_true', 
                        help='Run without display window (for SSH/headless operation)')
    parser.add_argument('--camera', type=int, default=0, 
                        help='Camera index (default: 0)')
    parser.add_argument('--width', type=int, default=640, 
                        help='Camera width (default: 640)')
    parser.add_argument('--height', type=int, default=480, 
                        help='Camera height (default: 480)')
    parser.add_argument('--serial', type=str, default=SERIAL_PORT,
                        help=f'Serial port (default: {SERIAL_PORT})')
    parser.add_argument('--baud', type=int, default=BAUD_RATE,
                        help=f'Baud rate (default: {BAUD_RATE})')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging')
    args = parser.parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")

    # 1) Initialize serial connection to motor controller
    try:
        ser = serial.Serial(args.serial, baudrate=args.baud, timeout=SERIAL_TIMEOUT)
        time.sleep(2)  # Wait for device initialization
        logging.info(f"Serial connection established: {args.serial} @ {args.baud} baud")
        stop_motors(ser)  # Start with motors stopped
    except serial.SerialException as e:
        logging.error(f"Failed to open serial port {args.serial}: {e}")
        sys.exit(1)

    # 2) Initialize ArUco dictionary
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    elif hasattr(cv2.aruco, 'Dictionary_get'):
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    elif hasattr(cv2.aruco, 'Dictionary'):
        try:
            aruco_dict = cv2.aruco.Dictionary(cv2.aruco.DICT_4X4_50)
        except Exception:
            raise RuntimeError("Unable to obtain ArUco dictionary")
    else:
        raise RuntimeError("cv2.aruco does not expose a known dictionary getter")

    # 3) Detector parameters
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        parameters = cv2.aruco.DetectorParameters_create()
    elif hasattr(cv2.aruco, 'DetectorParameters'):
        parameters = cv2.aruco.DetectorParameters()
    else:
        raise RuntimeError("cv2.aruco does not expose DetectorParameters")

    # 4) Initialize video capture
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        logging.error(f"Cannot open camera (index {args.camera}). Exiting.")
        stop_motors(ser)
        ser.close()
        sys.exit(1)

    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    logging.info(f"Camera initialized: {frame_width}x{frame_height}")
    logging.info(f"Headless mode: {args.headless}")
    logging.info(f"Center tolerance: ±{CENTER_TOLERANCE}px, Min marker area: {MIN_MARKER_AREA}px²")
    if not args.headless:
        logging.info("Press 'q' to quit.")

    last_action = "Stop (no marker)"
    
    try:
        while True:
            # Read frame from camera
            ret, frame = cap.read()
            if not ret or frame is None:
                logging.error("Failed to read frame. Exiting.")
                break

            # Convert to grayscale for detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect markers
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # Process detection results
            if corners is not None and len(corners) > 0:
                # Draw detected markers
                cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))

                # Get first marker's center and area
                first = np.array(corners[0])
                pts = first.reshape((-1, 2))
                center = pts.mean(axis=0)
                cx, cy = center
                center_pt = (int(round(cx)), int(round(cy)))

                # Calculate marker area
                marker_area = cv2.contourArea(pts)

                # Draw center point
                cv2.circle(frame, center_pt, radius=6, color=(0, 0, 255), thickness=-1)

                # Draw frame center for reference
                frame_center = (frame_width // 2, frame_height // 2)
                cv2.circle(frame, frame_center, radius=4, color=(255, 0, 0), thickness=-1)
                cv2.line(frame, (frame_center[0] - 20, frame_center[1]), 
                        (frame_center[0] + 20, frame_center[1]), (255, 0, 0), 2)
                cv2.line(frame, (frame_center[0], frame_center[1] - 20), 
                        (frame_center[0], frame_center[1] + 20), (255, 0, 0), 2)

                # Compute motor command based on marker position
                left_power, right_power, action = compute_motor_command(cx, frame_width, marker_area)
                send_motor_command(ser, left_power, right_power)

                # Display info on frame
                marker_id = ids[0][0] if ids is not None else "?"
                info_text = [
                    f"ID: {marker_id}",
                    f"Center: ({center_pt[0]}, {center_pt[1]})",
                    f"Area: {int(marker_area)}",
                    f"Action: {action}"
                ]
                y_offset = 30
                for text in info_text:
                    cv2.putText(frame, text, (10, y_offset), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    y_offset += 30

                if action != last_action:
                    logging.info(f"Marker ID {marker_id} | Center: {center_pt} | Area: {int(marker_area)} | Action: {action}")
                    last_action = action

            else:
                # No marker detected - stop motors
                stop_motors(ser)
                if last_action != "Stop (no marker)":
                    logging.info("No marker detected - motors stopped")
                    last_action = "Stop (no marker)"
                
                # Display status
                cv2.putText(frame, "No marker detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Display frame (if not headless)
            if not args.headless:
                cv2.imshow("ArUco Motor Control", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    logging.info("'q' pressed — exiting.")
                    break
            else:
                cv2.waitKey(1)

    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received — exiting.")
    finally:
        # Cleanup
        stop_motors(ser)
        ser.close()
        cap.release()
        cv2.destroyAllWindows()
        logging.info("Motors stopped, serial closed, camera released. Goodbye.")


if __name__ == "__main__":
    main()
