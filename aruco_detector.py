"""
ArUco Detector

Requirements (install before running):
    python -m pip install opencv-python opencv-contrib-python numpy

This script:
 - Uses the ArUco dictionary DICT_4X4_50 (value 22) and marker size 200 mm (0.2 m)
 - Opens the default webcam (index 0)
 - Detects markers, draws a green bounding box and ID, computes the center of the
   first detected marker, draws a red circle on that center and overlays the
   coordinates as text. Press 'q' to quit.

Notes / Improvements included:
 - Basic logging and robust frame-read handling
 - Placeholder variables and example code for pose estimation using camera
   calibration (camera matrix and distortion coefficients). If you provide
   calibration data, the script will estimate pose (translation/rotation) and
   can draw axes and distance in mm.

Author: Generated for user
"""

import sys
import logging
import cv2
import numpy as np


def main():
    # Basic logger
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    # 1) Choose the ArUco dictionary: DICT_4X4_50 (value 22)
    #    This dictionary contains 50 markers of 4x4 bits.
    #    Different OpenCV versions expose different APIs, so try the common
    #    alternatives in order to be compatible across installs.
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    elif hasattr(cv2.aruco, 'Dictionary_get'):
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    elif hasattr(cv2.aruco, 'Dictionary'):
        # Some builds expose a Dictionary class; try constructing if possible.
        try:
            aruco_dict = cv2.aruco.Dictionary(cv2.aruco.DICT_4X4_50)
        except Exception:
            raise RuntimeError("Unable to obtain ArUco dictionary from cv2.aruco in this OpenCV build")
    else:
        raise RuntimeError("cv2.aruco does not expose a known dictionary getter (getPredefinedDictionary/Dictionary_get/Dictionary)")

    # 2) Detector parameters (defaults are fine; you can tweak these)
    #    Newer OpenCV versions use DetectorParameters() constructor instead of _create()
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        parameters = cv2.aruco.DetectorParameters_create()
    elif hasattr(cv2.aruco, 'DetectorParameters'):
        parameters = cv2.aruco.DetectorParameters()
    else:
        raise RuntimeError("cv2.aruco does not expose DetectorParameters_create or DetectorParameters")

    # 3) Marker physical size (used for pose estimation) -- user specified 200 mm
    marker_size_m = 0.200  # meters (200 mm)

    # 4) Optional camera calibration (camera matrix and distortion coefficients)
    #    If you have camera calibration, set camera_matrix and dist_coeffs to
    #    the NumPy arrays obtained from calibration. Leave as None to skip pose.
    #    Example formats:
    #    camera_matrix = np.array([[fx, 0, cx],[0, fy, cy],[0,0,1]], dtype=float)
    #    dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=float)
    camera_matrix = None
    dist_coeffs = None

    # 5) Initialize video capture (default webcam index 0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logging.error("Cannot open default camera (index 0). Exiting.")
        sys.exit(1)

    logging.info("Starting video capture. Press 'q' to quit.")

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
            #    returns: corners (list), ids (ndarray or None), rejectedCandidates
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # 9) Check if any markers were detected
            if corners is not None and len(corners) > 0:
                # 10) Draw detected markers (green bounding boxes and IDs) on original frame
                #     drawDetectedMarkers accepts the color frame and the result corners
                cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))

                # 11) For the first detected marker, compute its center by averaging its 4 corners
                #    corners[0] may have shape (1,4,2) or (4,1,2) depending on OpenCV
                first = np.array(corners[0])
                # reshape to (4,2)
                pts = first.reshape((-1, 2))
                center = pts.mean(axis=0)
                cx, cy = center

                # 12) Draw a red circle at the center coordinate on the frame
                center_pt = (int(round(cx)), int(round(cy)))
                cv2.circle(frame, center_pt, radius=6, color=(0, 0, 255), thickness=-1)

                # 13) Put text showing the (x, y) coordinates of the center
                coords_text = f"Center: ({center_pt[0]}, {center_pt[1]})"
                cv2.putText(frame, coords_text, (center_pt[0] + 10, center_pt[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # --- Optional pose estimation if calibration data is provided ---
                if camera_matrix is not None and dist_coeffs is not None:
                    # estimatePoseSingleMarkers returns rvecs, tvecs, and _
                    # markerLength must be in the same unit as camera_matrix (meters here)
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size_m, camera_matrix, dist_coeffs)

                    # draw pose axes for the first marker (length = marker_size_m * 0.5)
                    if rvecs is not None and len(rvecs) > 0:
                        rvec, tvec = rvecs[0], tvecs[0]
                        cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size_m * 0.5)

                        # tvec is translation vector from camera to marker in meters
                        distance_m = np.linalg.norm(tvec)
                        distance_mm = distance_m * 1000.0
                        dist_text = f"Distance: {distance_mm:.1f} mm"
                        cv2.putText(frame, dist_text, (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (255, 0, 0), 2)

            # 14) Display the processed frame
            cv2.imshow("ArUco Detector", frame)

            # 15) Wait for 'q' key to be pressed to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logging.info("'q' pressed — exiting.")
                break

    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received — exiting.")
    finally:
        # 16) Release camera and destroy any OpenCV windows
        cap.release()
        cv2.destroyAllWindows()
        logging.info("Camera released and windows destroyed. Goodbye.")


if __name__ == "__main__":
    main()
