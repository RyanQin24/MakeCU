"""
Blue Bin Motor Control Integration Script - Raspberry Pi

Requirements (install on RPi before running):
    sudo apt-get install python3-opencv python3-numpy python3-serial
    # OR via pip:
    pip3 install opencv-python opencv-contrib-python numpy pyserial

This script:
 - Detects blue bin using USB webcam and color detection
 - Centers the bin in the camera view by sending motor commands via serial
 - Movement logic:
   * If bin is left of center → turn left
   * If bin is right of center → turn right
   * If bin is relatively centered → move forward
   * No bin detected → stop motors (fallback to manual control)
 - Uses the same motor power values and serial device as aruco_motor_control.py
 - Can run headless (no display) for production use

Control Strategy:
 - CENTER_TOLERANCE: pixel range for "centered" (default ±50 pixels)
 - FORWARD_THRESHOLD: minimum bin size to move forward (prevents moving when far)
 - Motor commands sent via serial to Arduino/motor controller at /dev/ttyACM1

Author: Generated for MakeCU project
"""

import sys
import logging
import argparse
import time
import serial
from blue_bin_detector import BlueBinDetector


# Motor configuration (from aruco_motor_control.py)
FORWARD_LEFT_POWER = 260
FORWARD_RIGHT_POWER = -260
LEFT_L_POWER = 260
LEFT_R_POWER = 260

# Serial configuration - PORT 1 FOR MOTORS
SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 57600
SERIAL_TIMEOUT = 1

# Detection and control parameters
CENTER_TOLERANCE = 50  # pixels - if bin is within ±50px of center, consider it centered
TURN_SPEED_DIVISOR = 2  # reduce turn speed for smoother centering
MIN_BIN_DISTANCE = 0.15  # minimum distance estimate (0-1) to consider bin close enough


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


def compute_motor_command(angle, distance_estimate, frame_width):
    """
    Compute motor commands based on bin angle and distance.
    
    Args:
        angle: horizontal angle to bin (-90 to +90 degrees, negative=left, positive=right)
        distance_estimate: bin size estimate (0-1, higher = closer)
        frame_width: width of the camera frame (for reference)
    
    Returns:
        tuple: (left_power, right_power, action_description)
    """
    # Convert angle to approximate pixel offset for consistency with ArUco logic
    # Assuming 60° FOV, frame_center is frame_width/2
    # angle range: -30° to +30° (half of 60° FOV)
    frame_center = frame_width / 2.0
    # Scale angle to pixel offset (approximate)
    pixel_offset_estimate = (angle / 30.0) * frame_center
    
    # Check if bin is centered horizontally (using pixel-equivalent tolerance)
    if abs(pixel_offset_estimate) <= CENTER_TOLERANCE:
        # Bin is centered - move forward if bin is close enough
        if distance_estimate >= MIN_BIN_DISTANCE:
            return (FORWARD_LEFT_POWER, FORWARD_RIGHT_POWER, "Forward (centered)")
        else:
            # Bin too small/far - just stop
            return (0, 0, "Stop (bin too far)")
    
    # Bin is off-center - turn to center it
    elif angle < 0:
        # Bin is to the left of center - turn left
        turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
        turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
        return (-turn_left, -turn_right, "Turn Left")
    else:
        # Bin is to the right of center - turn right
        turn_left = LEFT_L_POWER // TURN_SPEED_DIVISOR
        turn_right = LEFT_R_POWER // TURN_SPEED_DIVISOR
        return (turn_left, turn_right, "Turn Right")


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Blue Bin Motor Control for Raspberry Pi')
    parser.add_argument('--headless', action='store_true', 
                        help='Run without display window (for SSH/headless operation)')
    parser.add_argument('--camera', type=int, default=0, 
                        help='Camera index (default: 0)')
    parser.add_argument('--serial', type=str, default=SERIAL_PORT,
                        help=f'Serial port (default: {SERIAL_PORT})')
    parser.add_argument('--baud', type=int, default=BAUD_RATE,
                        help=f'Baud rate (default: {BAUD_RATE})')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging and camera visualization')
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

    # 2) Initialize Blue Bin Detector
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
        logging.info(f"Center tolerance: ±{CENTER_TOLERANCE}px equivalent, Min distance: {MIN_BIN_DISTANCE}")
        if not args.headless:
            logging.info("Press Ctrl+C to quit.")
        
    except Exception as e:
        logging.error(f"Failed to initialize Blue Bin Detector: {e}")
        stop_motors(ser)
        ser.close()
        sys.exit(1)

    last_action = "Stop (no bin)"
    frame_count = 0
    
    try:
        while True:
            frame_count += 1
            
            # Detect blue bin
            detection = detector.detect_blue_bin()

            # Process detection results
            if detection is not None:
                angle, distance_estimate, confidence = detection
                
                # Compute motor command based on bin position
                left_power, right_power, action = compute_motor_command(
                    angle, distance_estimate, frame_width
                )
                send_motor_command(ser, left_power, right_power)

                # Log info periodically or when action changes
                if action != last_action or frame_count % 30 == 0:
                    logging.info(
                        f"Bin detected | Angle: {angle:+.1f}° | "
                        f"Distance: {distance_estimate:.2f} | "
                        f"Confidence: {confidence:.2f} | "
                        f"Action: {action}"
                    )
                    last_action = action

            else:
                # No bin detected - stop motors
                stop_motors(ser)
                if last_action != "Stop (no bin)":
                    logging.info("No bin detected - motors stopped")
                    last_action = "Stop (no bin)"

            # Small delay to prevent overwhelming the system
            time.sleep(0.05)

    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received — exiting.")
    finally:
        # Cleanup
        stop_motors(ser)
        ser.close()
        detector.cleanup()
        
        # Print statistics
        stats = detector.get_statistics()
        logging.info("\n" + "="*60)
        logging.info("STATISTICS")
        logging.info("="*60)
        logging.info(f"Total frames processed: {stats['total_frames']}")
        logging.info(f"Frames with bin detected: {stats['detected_frames']}")
        logging.info(f"Detection rate: {stats['detection_rate']:.1f}%")
        logging.info("="*60)
        logging.info("Motors stopped, serial closed, camera released. Goodbye.")


if __name__ == "__main__":
    main()
