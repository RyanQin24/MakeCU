"""
Test blue bin detection with camera
"""

import time
from blue_bin_detector import BlueBinDetector


def test_camera():
    """Test camera and blue bin detection."""
    
    print("\n" + "=" * 60)
    print("BLUE BIN DETECTION TEST")
    print("=" * 60)
    print("\nTesting camera and blue color detection")
    print("Press Ctrl+C to stop\n")
    
    detector = None
    
    try:
        # Initialize with debug ON to see visualization
        detector = BlueBinDetector(camera_index=0, debug=True)
        
        print("Camera active - point at blue object")
        print("-" * 60)
        
        frame_count = 0
        
        while True:
            frame_count += 1
            
            # Detect blue bin
            detection = detector.detect_blue_bin()
            
            if detection:
                angle, distance_est, confidence = detection
                
                # Show detection result
                print(f"[{frame_count:4d}] "
                      f"✓ DETECTED | "
                      f"Angle: {angle:+6.1f}° | "
                      f"Size: {distance_est:.2f} | "
                      f"Confidence: {confidence:.2f}")
            else:
                print(f"[{frame_count:4d}] ✗ Not detected", end='\r')
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nTest stopped")
    
    finally:
        if detector:
            stats = detector.get_statistics()
            print("\n" + "=" * 60)
            print("DETECTION STATISTICS")
            print("=" * 60)
            print(f"Total frames: {stats['total_frames']}")
            print(f"Detected frames: {stats['detected_frames']}")
            print(f"Detection rate: {stats['detection_rate']:.1f}%")
            print("=" * 60)
            
            detector.cleanup()


if __name__ == "__main__":
    test_camera()