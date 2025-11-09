"""
Blue Garbage Bin Detector using Camera
Detects blue bin and calculates angle to it
"""

import cv2
import numpy as np
import time
from typing import Optional, Tuple


class BlueBinDetector:
    """
    Detect blue garbage bin using camera color detection.
    """
    
    def __init__(self, camera_index: int = 0, debug: bool = False):
        """
        Initialize blue bin detector.
        
        Args:
            camera_index: Camera device index (0 = default)
            debug: Show debug windows with detection visualization
        """
        self.debug = debug
        
        print("\n" + "=" * 60)
        print("BLUE BIN DETECTOR - CAMERA VISION")
        print("=" * 60)
        
        # Import config
        try:
            import config_vision as config
        except ImportError:
            print("ERROR: config_vision.py not found!")
            raise
        
        self.config = config
        
        # Initialize camera
        print(f"\nInitializing camera {camera_index}...")
        self.cap = cv2.VideoCapture(camera_index)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {camera_index}")
        
        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        
        # Warm up camera
        for _ in range(5):
            self.cap.read()
            time.sleep(0.1)
        
        print("✓ Camera initialized")
        print(f"  Resolution: {config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT}")
        print(f"  Debug mode: {'ON' if debug else 'OFF'}")
        print("=" * 60 + "\n")
        
        # Detection statistics
        self.total_frames = 0
        self.detected_frames = 0
    
    def detect_blue_bin(self) -> Optional[Tuple[float, float, float, float]]:
        """
        Detect blue bin in current camera frame.
        
        Returns:
            Tuple of (angle, distance_estimate, confidence, blue_ratio) or None
            - angle: Horizontal angle to bin center (-90 to +90 degrees)
              Negative = left, Positive = right
            - distance_estimate: Rough distance based on bin size (0-1)
              Larger bin = closer (higher value)
            - confidence: Detection confidence (0-1)
            - blue_ratio: Percentage of frame covered by blue bin (0-1)
              Used to detect when bin is reached (>0.9 = touching bin)
        """
        # Read frame
        ret, frame = self.cap.read()
        
        if not ret:
            return None
        
        self.total_frames += 1
        
        # Convert to HSV color space (better for color detection)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for blue color
        mask = cv2.inRange(
            hsv,
            self.config.BLUE_LOWER_HSV,
            self.config.BLUE_UPPER_HSV
        )
        
        # Clean up mask (remove noise)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        if not contours:
            if self.debug:
                self._show_debug(frame, mask, None)
            return None
        
        # Find largest contour (should be the bin)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Check if area is large enough
        if area < self.config.MIN_BIN_AREA:
            if self.debug:
                self._show_debug(frame, mask, None)
            return None
        
        # Calculate bounding box
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Calculate center of bin
        bin_center_x = x + w // 2
        bin_center_y = y + h // 2
        
        # Calculate angle to bin
        # Camera center is at CAMERA_WIDTH/2
        # Negative angle = left, Positive = right
        frame_center = self.config.CAMERA_WIDTH / 2
        pixel_offset = bin_center_x - frame_center
        
        # Convert pixel offset to angle
        # Assuming ~60° field of view horizontally
        fov = 60.0  # degrees
        angle = (pixel_offset / frame_center) * (fov / 2)
        
        # Calculate distance estimate based on bin size
        # Larger bin = closer
        max_area = self.config.CAMERA_WIDTH * self.config.CAMERA_HEIGHT * 0.5
        distance_estimate = min(1.0, area / max_area)
        
        # Calculate blue_ratio: what percentage of frame is covered by blue bin
        frame_area = self.config.CAMERA_WIDTH * self.config.CAMERA_HEIGHT
        blue_ratio = area / frame_area
        
        # Calculate confidence based on shape
        # Perfect rectangle = high confidence
        rect_area = w * h
        shape_match = area / rect_area if rect_area > 0 else 0
        confidence = min(1.0, shape_match)
        
        self.detected_frames += 1
        
        # Show debug visualization
        if self.debug:
            debug_info = {
                'contour': largest_contour,
                'bbox': (x, y, w, h),
                'center': (bin_center_x, bin_center_y),
                'angle': angle,
                'area': area,
                'blue_ratio': blue_ratio
            }
            self._show_debug(frame, mask, debug_info)
        
        return (angle, distance_estimate, confidence, blue_ratio)
    
    def _show_debug(self, frame, mask, detection_info):
        """Show debug visualization windows."""
        
        debug_frame = frame.copy()
        
        if detection_info:
            # Draw bounding box
            x, y, w, h = detection_info['bbox']
            cv2.rectangle(debug_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Draw center point
            cx, cy = detection_info['center']
            cv2.circle(debug_frame, (cx, cy), 5, (0, 0, 255), -1)
            
            # Draw center line
            frame_center = self.config.CAMERA_WIDTH // 2
            cv2.line(debug_frame, (frame_center, 0), 
                    (frame_center, self.config.CAMERA_HEIGHT), (255, 0, 0), 1)
            
            # Add text
            angle = detection_info['angle']
            area = detection_info['area']
            blue_ratio = detection_info.get('blue_ratio', 0.0)
            cv2.putText(debug_frame, f"Angle: {angle:+.1f} deg", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_frame, f"Area: {area:.0f} px", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_frame, f"Blue: {blue_ratio*100:.1f}%", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Show frames
        cv2.imshow("Camera View", debug_frame)
        cv2.imshow("Blue Mask", mask)
        cv2.waitKey(1)
    
    def get_statistics(self) -> dict:
        """Get detection statistics."""
        detection_rate = (self.detected_frames / self.total_frames * 100) \
                        if self.total_frames > 0 else 0
        
        return {
            'total_frames': self.total_frames,
            'detected_frames': self.detected_frames,
            'detection_rate': detection_rate
        }
    
    def cleanup(self):
        """Release camera and close windows."""
        self.cap.release()
        if self.debug:
            cv2.destroyAllWindows()
        print("\nCamera released")