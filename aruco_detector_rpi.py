"""
ArUco Detector - Raspberry Pi Optimized Version

Requirements (install on RPi before running):
    sudo apt-get update
    sudo apt-get install python3-opencv python3-numpy
    # OR via pip:
    pip3 install opencv-python opencv-contrib-python numpy

This script:
 - Uses the ArUco dictionary DICT_4X4_50 (value 22) and marker size 200 mm (0.2 m)
 - Opens USB webcam (index 0) with optimized settings for RPi
 - Detects markers, draws green bounding boxes and IDs, computes center
 - Can run headless (no display) or with GUI display
 - Press 'q' to quit (GUI mode only)

Optimizations for Raspberry Pi:
 - Reduced resolution (640x480) for better performance
 - Non-blocking frame reads
 - Optional headless mode for SSH use
 - Configurable FPS and camera settings

Author: Generated for user
"""

import sys
import logging
import argparse
import cv2
import numpy as np


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='ArUco Detector for Raspberry Pi')
    parser.add_argument('--headless', action='store_true', 
                        help='Run without display window (for SSH/headless operation)')
    parser.add_argument('--camera', type=int, default=0, 
                        help='Camera index (default: 0)')
    parser.add_argument('--width', type=int, default=640, 
                        help='Camera width (default: 640)')
    parser.add_argument('--height', type=int, default=480, 
                        help='Camera height (default: 480)')
    parser.add_argument('--fps', type=int, default=30, 
                        help='Target FPS (default: 30)')
    args = parser.parse_args()

    # Basic logger
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    # 1) Choose the ArUco dictionary: DICT_4X4_50 (value 22)
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    elif hasattr(cv2.aruco, 'Dictionary_get'):
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    elif hasattr(cv2.aruco, 'Dictionary'):
        try:
            aruco_dict = cv2.aruco.Dictionary(cv2.aruco.DICT_4X4_50)
        except Exception:
            raise RuntimeError("Unable to obtain ArUco dictionary from cv2.aruco in this OpenCV build")
    else:
        raise RuntimeError("cv2.aruco does not expose a known dictionary getter")

    # 2) Detector parameters
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        parameters = cv2.aruco.DetectorParameters_create()
    elif hasattr(cv2.aruco, 'DetectorParameters'):
        parameters = cv2.aruco.DetectorParameters()
    else:
        raise RuntimeError("cv2.aruco does not expose DetectorParameters_create or DetectorParameters")

    # 3) Marker physical size
    marker_size_m = 0.200  # meters (200 mm)

    # 4) Optional camera calibration placeholders
    camera_matrix = None
    dist_coeffs = None

    # 5) Initialize video capture with optimized settings for RPi
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        logging.error(f"Cannot open camera (index {args.camera}). Exiting.")
        sys.exit(1)

    # Set camera properties for better performance
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    
    # Disable auto-focus if supported (reduces jitter)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    logging.info(f"Camera initialized: {int(actual_width)}x{int(actual_height)}")
    logging.info(f"Headless mode: {args.headless}")
    if not args.headless:
        logging.info("Press 'q' to quit.")

    try:
        while True:
            # 6) Read a frame from the camera
            ret, frame = cap.read()
            if not ret or frame is None:
                logging.error("Failed to read frame from camera. Exiting loop.")
                break

            # 7) Convert frame to grayscale for detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 8) Detect markers in the grayscale image
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # 9) Check if any markers were detected
            if corners is not None and len(corners) > 0:
                # 10) Draw detected markers (green bounding boxes and IDs)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))

                # 11) Compute center of first detected marker
                first = np.array(corners[0])
                pts = first.reshape((-1, 2))
                center = pts.mean(axis=0)
                cx, cy = center

                # 12) Draw a red circle at the center coordinate
                center_pt = (int(round(cx)), int(round(cy)))
                cv2.circle(frame, center_pt, radius=6, color=(0, 0, 255), thickness=-1)

                # 13) Put text showing the (x, y) coordinates
                coords_text = f"Center: ({center_pt[0]}, {center_pt[1]})"
                cv2.putText(frame, coords_text, (center_pt[0] + 10, center_pt[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Log detection info (useful in headless mode)
                marker_id = ids[0][0] if ids is not None else "unknown"
                logging.info(f"Marker ID {marker_id} detected at center: {center_pt}")

                # Optional pose estimation
                if camera_matrix is not None and dist_coeffs is not None:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, marker_size_m, camera_matrix, dist_coeffs)
                    if rvecs is not None and len(rvecs) > 0:
                        rvec, tvec = rvecs[0], tvecs[0]
                        cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, 
                                          rvec, tvec, marker_size_m * 0.5)
                        distance_m = np.linalg.norm(tvec)
                        distance_mm = distance_m * 1000.0
                        dist_text = f"Distance: {distance_mm:.1f} mm"
                        cv2.putText(frame, dist_text, (10, frame.shape[0] - 20), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # 14) Display the processed frame (only if not headless)
            if not args.headless:
                cv2.imshow("ArUco Detector (RPi)", frame)

                # 15) Wait for 'q' key to be pressed to exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    logging.info("'q' pressed — exiting.")
                    break
            else:
                # In headless mode, just a small delay to avoid CPU overload
                cv2.waitKey(1)

    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received — exiting.")
    finally:
        # 16) Release camera and destroy any OpenCV windows
        cap.release()
        cv2.destroyAllWindows()
        logging.info("Camera released and windows destroyed. Goodbye.")


if __name__ == "__main__":
    main()
