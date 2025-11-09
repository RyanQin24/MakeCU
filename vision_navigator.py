"""
Vision-Guided Navigation System
Finds blue bin and navigates to it while avoiding obstacles
"""

import time
import math
from enum import Enum
from typing import Optional, Tuple
from blue_bin_detector import BlueBinDetector
from ultrasonic_sensor import UltrasonicSensor
from motor_controller import MotorController
import config_vision as config


class NavigationState(Enum):
    """Navigation states."""
    SEARCH = 1      # Rotating 360° to find blue bin
    NAVIGATE = 2    # Moving toward blue bin
    AVOID = 3       # Avoiding obstacle
    REALIGN = 4     # Realigning to target after avoidance
    REACHED = 5     # Reached target


class VisionNavigator:
    """
    Vision-guided navigation with obstacle avoidance.
    
    Algorithm:
    1. SEARCH: Rotate 360° to find blue bin, record angle
    2. NAVIGATE: Move toward bin using recorded angle
    3. AVOID: If obstacle detected, avoid it
    4. REALIGN: After avoidance, re-detect bin and update angle
    5. REACHED: Stop when close to bin
    """
    
    def __init__(self, 
                 sensor_port: str = None,
                 camera_index: int = None,
                 debug: bool = False):
        """Initialize vision navigator."""
        
        print("\n" + "=" * 70)
        print("VISION-GUIDED NAVIGATOR")
        print("Blue Bin Detection + Obstacle Avoidance")
        print("=" * 70)
        
        if sensor_port is None:
            sensor_port = config.SENSOR_PORT
        if camera_index is None:
            camera_index = config.CAMERA_INDEX
        
        # Initialize camera
        self.camera = BlueBinDetector(camera_index=camera_index, debug=debug)
        
        # Initialize ultrasonic sensor
        self.sensor = UltrasonicSensor(
            port=sensor_port,
            baudrate=config.BAUDRATE,
            debug=False
        )
        
        # Initialize motors
        self.motors = MotorController()
        
        # Navigation state
        self.state = NavigationState.SEARCH
        self.running = False
        
        # Target tracking
        self.target_angle = None         # Absolute angle to target (degrees)
        self.current_heading = 0.0       # Robot's current heading (degrees)
        self.target_found = False
        
        # Search tracking
        self.search_rotation = 0.0       # Total rotation during search
        
        # Avoidance tracking
        self.avoid_attempts = 0
        self.pre_avoid_heading = None
        
        # Frame counter
        self.frame_count = 0
        
        print("\n" + "=" * 70)
        print("✓ Vision Navigator Ready!")
        print("=" * 70 + "\n")
    
    def search_for_target(self) -> bool:
        """
        Rotate 360° to search for blue bin.
        
        Returns:
            True if target found, False otherwise
        """
        print("\n" + "=" * 70)
        print("SEARCHING FOR BLUE BIN (360° rotation)")
        print("=" * 70)
        
        self.search_rotation = 0.0
        best_detection = None
        best_angle = None
        
        # Rotate 360° in steps
        while self.search_rotation < 360:
            # Check camera
            detection = self.camera.detect_blue_bin()
            
            if detection:
                angle_offset, distance_est, confidence = detection
                
                # Calculate absolute angle to target
                absolute_angle = self.current_heading + angle_offset
                
                print(f"  [DETECTED] At heading {self.current_heading:.1f}°: "
                      f"Bin at {angle_offset:+.1f}° offset "
                      f"(confidence: {confidence:.2f})")
                
                # Keep best detection (highest confidence)
                if best_detection is None or confidence > best_detection[2]:
                    best_detection = detection
                    best_angle = absolute_angle
            
            # Rotate a bit
            self.motors.turn_right(config.SEARCH_ROTATION_SPEED)
            time.sleep(config.SEARCH_ROTATION_DURATION)
            
            # Update rotation tracking
            rotation_step = config.SEARCH_STEP_ANGLE
            self.search_rotation += rotation_step
            self.current_heading += rotation_step
            self.current_heading = self.current_heading % 360
            
            print(f"  Search progress: {self.search_rotation:.0f}°/360°", end='\r')
        
        self.motors.stop()
        print()  # New line after progress
        
        if best_detection:
            self.target_angle = best_angle
            self.target_found = True
            print(f"\n✓ BLUE BIN FOUND at {self.target_angle:.1f}°")
            print("=" * 70 + "\n")
            return True
        else:
            print(f"\n✗ BLUE BIN NOT FOUND after 360° search")
            print("=" * 70 + "\n")
            return False
    
    def calculate_turn_to_target(self) -> float:
        """
        Calculate how much to turn to face target.
        
        Returns:
            Turn angle in degrees (negative = left, positive = right)
        """
        if self.target_angle is None:
            return 0.0
        
        # Calculate shortest angle difference
        angle_diff = self.target_angle - self.current_heading
        
        # Normalize to -180 to +180
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        return angle_diff
    
    def align_to_target(self):
        """Rotate robot to face target."""
        turn_needed = self.calculate_turn_to_target()
        
        if abs(turn_needed) < config.ANGLE_TOLERANCE:
            return  # Already aligned
        
        print(f"  Aligning to target: turning {turn_needed:+.1f}°")
        
        # Determine turn direction
        if turn_needed < 0:
            # Turn left
            self.motors.turn_left(config.TURN_SPEED)
        else:
            # Turn right
            self.motors.turn_right(config.TURN_SPEED)
        
        # Calculate turn duration (rough estimate)
        # Assume ~90° per second at TURN_SPEED
        turn_duration = abs(turn_needed) / 90.0
        time.sleep(turn_duration)
        
        self.motors.stop()
        
        # Update heading
        self.current_heading = self.target_angle
    
    def navigate_to_target(self):
        """
        Navigate toward target while checking for obstacles.
        
        Returns:
            'reached', 'obstacle', or 'continue'
        """
        # Check if we can see the target to update angle
        detection = self.camera.detect_blue_bin()
        
        if detection:
            angle_offset, distance_est, confidence = detection
            
            # Update target angle
            self.target_angle = self.current_heading + angle_offset
            
            # Check if we're close enough (large bin = close)
            if distance_est > 0.3:  # Bin takes up >30% of frame
                return 'reached'
        
        # Check for obstacles
        distance = self.sensor.read_distance()
        
        if distance is not None and distance < config.DANGER_DISTANCE:
            return 'obstacle'
        
        # Align to target
        turn_needed = self.calculate_turn_to_target()
        
        if abs(turn_needed) > config.ANGLE_TOLERANCE:
            # Need to turn
            if turn_needed < 0:
                self.motors.turn_left(config.TURN_SPEED)
            else:
                self.motors.turn_right(config.TURN_SPEED)
        else:
            # Move forward
            if distance is not None and distance < config.WARNING_DISTANCE:
                self.motors.move_forward(config.SLOW_SPEED)
            else:
                self.motors.move_forward(config.FULL_SPEED)
        
        return 'continue'
    
    def avoid_obstacle(self):
        """
        Avoid obstacle by turning and moving sideways.
        """
        print("\n  [OBSTACLE] Avoiding...")
        
        # Save current heading
        self.pre_avoid_heading = self.current_heading
        
        # Turn 45° right (or left if preferred)
        print(f"  Turning {config.AVOID_TURN_ANGLE}° to avoid")
        self.motors.turn_right(config.TURN_SPEED)
        time.sleep(config.AVOID_TURN_ANGLE / 90.0)  # Rough timing
        self.motors.stop()
        
        # Update heading
        self.current_heading += config.AVOID_TURN_ANGLE
        self.current_heading = self.current_heading % 360
        
        # Move forward to clear obstacle
        print(f"  Moving sideways for {config.AVOID_MOVE_DURATION}s")
        self.motors.move_forward(config.AVOID_SPEED)
        time.sleep(config.AVOID_MOVE_DURATION)
        self.motors.stop()
        
        # Check if obstacle cleared
        distance = self.sensor.read_distance()
        if distance is not None and distance > config.SAFE_DISTANCE:
            print("  ✓ Obstacle cleared!")
            return True
        else:
            print("  ⚠ Obstacle still present")
            self.avoid_attempts += 1
            return False
    
    def run(self, max_duration: float = None):
        """
        Run complete navigation mission.
        
        Args:
            max_duration: Maximum mission time in seconds (None = unlimited)
        """
        print("\n" + "=" * 70)
        print("STARTING VISION-GUIDED NAVIGATION MISSION")
        print("=" * 70)
        print("\nMission phases:")
        print("  1. SEARCH for blue bin (360° rotation)")
        print("  2. NAVIGATE to bin (straight line)")
        print("  3. AVOID obstacles when detected")
        print("  4. REALIGN to target after avoidance")
        print("  5. REACHED target")
        print("\nPress Ctrl+C to stop\n")
        
        self.running = True
        self.state = NavigationState.SEARCH
        start_time = time.time()
        
        try:
            # PHASE 1: SEARCH
            if not self.search_for_target():
                print("Mission failed: Target not found")
                return
            
            self.state = NavigationState.NAVIGATE
            
            # PHASE 2-5: NAVIGATE with obstacle avoidance
            while self.running:
                current_time = time.time()
                
                # Check timeout
                if max_duration and (current_time - start_time) > max_duration:
                    print("\nMission timeout reached")
                    break
                
                self.frame_count += 1
                
                # State machine
                if self.state == NavigationState.NAVIGATE:
                    result = self.navigate_to_target()
                    
                    if result == 'reached':
                        self.state = NavigationState.REACHED
                        print("\n" + "=" * 70)
                        print("✓ TARGET REACHED!")
                        print("=" * 70)
                        break
                    
                    elif result == 'obstacle':
                        self.state = NavigationState.AVOID
                        self.avoid_attempts = 0
                
                elif self.state == NavigationState.AVOID:
                    success = self.avoid_obstacle()
                    
                    if success:
                        self.state = NavigationState.REALIGN
                    elif self.avoid_attempts >= config.MAX_AVOID_ATTEMPTS:
                        print("\n✗ Too many avoidance attempts - mission failed")
                        break
                
                elif self.state == NavigationState.REALIGN:
                    print("\n  [REALIGN] Re-detecting target...")
                    
                    # Try to detect target again
                    detection = self.camera.detect_blue_bin()
                    
                    if detection:
                        angle_offset, _, confidence = detection
                        self.target_angle = self.current_heading + angle_offset
                        print(f"  ✓ Target re-acquired at {self.target_angle:.1f}°")
                        self.state = NavigationState.NAVIGATE
                    else:
                        # Can't see target - rotate to search
                        print("  Target not visible, searching...")
                        self.motors.turn_right(config.SEARCH_ROTATION_SPEED)
                        time.sleep(0.5)
                        self.current_heading += 10
                
                # Display status
                distance = self.sensor.read_distance()
                dist_str = f"{distance:.0f}cm" if distance else "N/A"
                
                print(f"[{self.frame_count:4d}] "
                      f"State: {self.state.name:10s} | "
                      f"Heading: {self.current_heading:6.1f}° | "
                      f"Target: {self.target_angle:6.1f}° | "
                      f"Distance: {dist_str:>6s}")
                
                time.sleep(config.MIN_ACTION_DURATION)
        
        except KeyboardInterrupt:
            print("\n\nMission aborted by user")
        
        finally:
            self.motors.stop()
            
            # Statistics
            cam_stats = self.camera.get_statistics()
            sensor_stats = self.sensor.get_statistics()
            
            print("\n" + "=" * 70)
            print("MISSION STATISTICS")
            print("=" * 70)
            print(f"Total frames: {self.frame_count}")
            print(f"Final state: {self.state.name}")
            print(f"Camera detection rate: {cam_stats['detection_rate']:.1f}%")
            print(f"Sensor success rate: {sensor_stats['success_rate']:.1f}%")
            print("=" * 70)
    
    def cleanup(self):
        """Clean up all resources."""
        self.camera.cleanup()
        self.sensor.close()
        self.motors.cleanup()


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Run vision-guided navigation')
    parser.add_argument('--port', type=str, help='Ultrasonic sensor port')
    parser.add_argument('--camera', type=int, help='Camera index')
    parser.add_argument('--duration', type=float, help='Max duration (seconds)')
    parser.add_argument('--debug', action='store_true', help='Show camera debug windows')
    
    args = parser.parse_args()
    
    robot = None
    
    try:
        robot = VisionNavigator(
            sensor_port=args.port,
            camera_index=args.camera,
            debug=args.debug
        )
        
        robot.run(max_duration=args.duration)
        
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if robot:
            robot.cleanup()


if __name__ == "__main__":
    main()