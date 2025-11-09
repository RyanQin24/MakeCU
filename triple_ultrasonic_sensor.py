"""
Triple Ultrasonic Sensor System - Single Serial Port
Simplified version for obstacle detection
Format: "front,left,right" (e.g., "100,150,200")
"""

import serial
import time
import re
from typing import Dict, Optional


class TripleUltrasonicSensor:
    """
    Read 3 ultrasonic sensors from single serial port.
    Expects format: "front,left,right\n" (e.g., "100,150,200\n")
    """
    
    def __init__(self, 
                 port: str = "/dev/ttyACM0",
                 baudrate: int = 9600,
                 debug: bool = False):
        """
        Initialize triple sensor system.
        
        Args:
            port: Serial port for all sensors
            baudrate: Communication speed
            debug: Show raw data and parsing
        """
        self.debug = debug
        self.port = port
        self.baudrate = baudrate
        
        if debug:
            print("\n" + "=" * 60)
            print("TRIPLE ULTRASONIC SENSOR - SINGLE PORT")
            print("=" * 60)
            print(f"Connecting to {port} at {baudrate} baud...")
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.5,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Wait for connection to stabilize
            time.sleep(0.5)
            
            # Clear any old data
            self.serial.reset_input_buffer()
            
            if debug:
                print(f"✓ Connected successfully!")
                print("=" * 60 + "\n")
            
        except serial.SerialException as e:
            print(f"✗ ERROR: Could not connect to {port}: {e}")
            raise
        
        # Statistics
        self.total_readings = 0
        self.valid_readings = 0
        
        # Last valid readings as fallback
        self.last_valid = {
            'front': 200.0,
            'left': 200.0,
            'right': 200.0
        }
    
    def read_sensors(self) -> Dict[str, Optional[float]]:
        """
        Read all three sensors from single serial line.
        Expected format: "front,left,right\n" (e.g., "100,150,200\n")
        
        Returns:
            Dictionary with 'front', 'left', 'right' distances (cm)
        """
        try:
            # Clear buffer to prevent reading stale/corrupted data
            self.serial.reset_input_buffer()
            
            # Small delay to allow fresh data to arrive
            time.sleep(0.05)
            
            # Read line from serial
            line = self.serial.readline()
            
            if not line:
                if self.debug:
                    print("[TIMEOUT] No data received")
                return self.last_valid.copy()
            
            # Decode
            line_str = None
            for encoding in ['utf-8', 'ascii', 'latin-1']:
                try:
                    line_str = line.decode(encoding).strip()
                    break
                except:
                    continue
            
            if not line_str:
                if self.debug:
                    print("[DECODE ERROR] Could not decode line")
                return self.last_valid.copy()
            
            if self.debug:
                print(f"[RAW] '{line_str}'")
            
            self.total_readings += 1
            
            # Parse the line: "front,left,right"
            readings = self._parse_readings(line_str)
            
            if readings:
                self.valid_readings += 1
                # Update last valid readings
                self.last_valid = readings.copy()
                
                if self.debug:
                    print(f"[OK] Front:{readings['front']:.1f}cm "
                          f"Left:{readings['left']:.1f}cm "
                          f"Right:{readings['right']:.1f}cm")
                
                return readings
            else:
                if self.debug:
                    print(f"[PARSE ERROR] Could not parse: '{line_str}'")
                return self.last_valid.copy()
            
        except Exception as e:
            if self.debug:
                print(f"[EXCEPTION] {e}")
            return self.last_valid.copy()
    
    def _parse_readings(self, line: str) -> Optional[Dict[str, float]]:
        """
        Parse sensor readings from format: "front,left,right"
        """
        # Extract all numbers from the line
        numbers = re.findall(r'\d+\.?\d*', line)
        
        if len(numbers) >= 3:
            try:
                front = float(numbers[0])
                left = float(numbers[1])
                right = float(numbers[2])
                
                # Validate ranges (2-400cm is typical for ultrasonic)
                if (self._validate_distance(front) and 
                    self._validate_distance(left) and 
                    self._validate_distance(right)):
                    
                    return {
                        'front': front,
                        'left': left,
                        'right': right
                    }
            except (ValueError, IndexError):
                pass
        
        return None
    
    def _validate_distance(self, distance: float) -> bool:
        """Validate if distance is in reasonable range."""
        return 2.0 <= distance <= 400.0
    
    def get_statistics(self) -> Dict:
        """Get reading statistics."""
        success_rate = (self.valid_readings / self.total_readings * 100) \
                      if self.total_readings > 0 else 0
        
        return {
            'total_readings': self.total_readings,
            'valid_readings': self.valid_readings,
            'success_rate': success_rate
        }
    
    def close(self):
        """Close serial connection."""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            if self.debug:
                print("\nSensors disconnected")


def main():
    """Test the sensor system."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Test triple ultrasonic sensors')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=9600,
                        help='Baud rate (default: 9600)')
    parser.add_argument('--debug', action='store_true',
                        help='Show debug output')
    args = parser.parse_args()
    
    try:
        sensors = TripleUltrasonicSensor(
            port=args.port,
            baudrate=args.baud,
            debug=args.debug
        )
        
        print("\nReading sensors... (Press Ctrl+C to stop)\n")
        print(f"{'Frame':<8} {'Front':<10} {'Left':<10} {'Right':<10}")
        print("-" * 40)
        
        frame = 0
        while True:
            readings = sensors.read_sensors()
            frame += 1
            
            print(f"{frame:<8} "
                  f"{readings['front']:<10.1f} "
                  f"{readings['left']:<10.1f} "
                  f"{readings['right']:<10.1f}")
            
            time.sleep(0.5)  # Match the 0.5s delay from main script
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    finally:
        if 'sensors' in locals():
            stats = sensors.get_statistics()
            print("\n" + "=" * 40)
            print("STATISTICS")
            print("=" * 40)
            print(f"Total readings: {stats['total_readings']}")
            print(f"Valid readings: {stats['valid_readings']}")
            print(f"Success rate: {stats['success_rate']:.1f}%")
            print("=" * 40)
            sensors.close()


if __name__ == "__main__":
    main()
