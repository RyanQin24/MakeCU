"""
Motor Controller for Raspberry Pi
Controls DC motors via GPIO pins
"""

import time

# Try to import RPi.GPIO
try:
    import RPi.GPIO as GPIO
    RASPBERRY_PI = True
except (ImportError, RuntimeError):
    RASPBERRY_PI = False
    print("WARNING: RPi.GPIO not available - running in SIMULATION mode")


class MotorController:
    """
    Control robot motors via GPIO pins.
    """
    
    def __init__(self):
        """Initialize motor controller with GPIO pins."""
        
        # GPIO Pin Configuration (BCM numbering)
        # ADJUST THESE BASED ON YOUR WIRING!
        self.LEFT_MOTOR_FORWARD = 17
        self.LEFT_MOTOR_BACKWARD = 27
        self.LEFT_MOTOR_ENABLE = 22
        
        self.RIGHT_MOTOR_FORWARD = 23
        self.RIGHT_MOTOR_BACKWARD = 24
        self.RIGHT_MOTOR_ENABLE = 25
        
        # Motor speeds (0-100)
        self.full_speed = 60
        self.slow_speed = 35
        self.turn_speed = 45
        
        if RASPBERRY_PI:
            # Setup GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup motor control pins
            pins = [
                self.LEFT_MOTOR_FORWARD,
                self.LEFT_MOTOR_BACKWARD,
                self.RIGHT_MOTOR_FORWARD,
                self.RIGHT_MOTOR_BACKWARD
            ]
            
            for pin in pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
            
            # Setup PWM for speed control
            GPIO.setup(self.LEFT_MOTOR_ENABLE, GPIO.OUT)
            GPIO.setup(self.RIGHT_MOTOR_ENABLE, GPIO.OUT)
            
            self.left_pwm = GPIO.PWM(self.LEFT_MOTOR_ENABLE, 1000)
            self.right_pwm = GPIO.PWM(self.RIGHT_MOTOR_ENABLE, 1000)
            
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            
            print("✓ Motor controller initialized (GPIO mode)")
        else:
            print("✓ Motor controller initialized (SIMULATION mode)")
    
    def set_motor_speed(self, left_speed: int, right_speed: int):
        """
        Set motor speeds.
        
        Args:
            left_speed: -100 to 100 (negative = backward)
            right_speed: -100 to 100 (negative = backward)
        """
        if not RASPBERRY_PI:
            print(f"[SIM] Motors: Left={left_speed:+4d}, Right={right_speed:+4d}")
            return
        
        # Left motor
        if left_speed > 0:
            GPIO.output(self.LEFT_MOTOR_FORWARD, GPIO.HIGH)
            GPIO.output(self.LEFT_MOTOR_BACKWARD, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(abs(left_speed))
        elif left_speed < 0:
            GPIO.output(self.LEFT_MOTOR_FORWARD, GPIO.LOW)
            GPIO.output(self.LEFT_MOTOR_BACKWARD, GPIO.HIGH)
            self.left_pwm.ChangeDutyCycle(abs(left_speed))
        else:
            GPIO.output(self.LEFT_MOTOR_FORWARD, GPIO.LOW)
            GPIO.output(self.LEFT_MOTOR_BACKWARD, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(0)
        
        # Right motor
        if right_speed > 0:
            GPIO.output(self.RIGHT_MOTOR_FORWARD, GPIO.HIGH)
            GPIO.output(self.RIGHT_MOTOR_BACKWARD, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(abs(right_speed))
        elif right_speed < 0:
            GPIO.output(self.RIGHT_MOTOR_FORWARD, GPIO.LOW)
            GPIO.output(self.RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
            self.right_pwm.ChangeDutyCycle(abs(right_speed))
        else:
            GPIO.output(self.RIGHT_MOTOR_FORWARD, GPIO.LOW)
            GPIO.output(self.RIGHT_MOTOR_BACKWARD, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(0)
    
    def move_forward(self, speed: int = None):
        """Move robot forward."""
        speed = speed or self.full_speed
        self.set_motor_speed(speed, speed)
    
    def move_backward(self, speed: int = None):
        """Move robot backward."""
        speed = speed or self.full_speed
        self.set_motor_speed(-speed, -speed)
    
    def turn_left(self, speed: int = None):
        """Turn robot left (pivot)."""
        speed = speed or self.turn_speed
        self.set_motor_speed(-speed, speed)
    
    def turn_right(self, speed: int = None):
        """Turn robot right (pivot)."""
        speed = speed or self.turn_speed
        self.set_motor_speed(speed, -speed)
    
    def stop(self):
        """Stop all motors."""
        self.set_motor_speed(0, 0)
    
    def cleanup(self):
        """Clean up GPIO resources."""
        if RASPBERRY_PI:
            self.stop()
            self.left_pwm.stop()
            self.right_pwm.stop()
            GPIO.cleanup()
            print("Motor controller cleaned up")