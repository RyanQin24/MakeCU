"""
Ultrasonic Sensor Reader - FIXED VERSION
Actually reads continuously from serial port
"""

import serial
import time
import platform
from typing import Optional


class UltrasonicSensor:
    """
    Read distance from serial ultrasonic sensor - BLOCKING READ VERSION
    """
    
    def __init__(self, port: str = None, baudrate: int = 9600, debug: bool = False):
        """
        Initialize serial ultrasonic sensor.
        
        Args:
            port: Serial port (None = auto-detect)
            baudrate: Communication speed
            debug: Show raw sensor data
        """
        self.debug = debug
        
        # Auto-detect port if not specified
        if port is None:
            port = self._auto_detect_port()
        
        self.port = port
        self.baudrate = baudrate
        
        print(f"\nConnecting to ultrasonic sensor...")
        print(f"  Port: {port}")
        print(f"  Baudrate: {baudrate}")
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=2.0,  # 2 second timeout for blocking read
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Wait for sensor to stabilize
            time.sleep(1.0)
            
            # Clear any old data
            self.serial.reset_input_buffer()
            
            print(f"✓ Connected successfully!")
            if debug:
                print(f"✓ DEBUG MODE ENABLED\n")
            else:
                print()
            
        except serial.SerialException as e:
            print(f"✗ ERROR: Could not connect to {port}")
            print(f"  {e}\n")
            self._print_troubleshooting()
            raise
        
        # Statistics
        self.total_readings = 0
        self.valid_readings = 0
        self.parse_errors = 0
    
    def _auto_detect_port(self) -> str:
        """Auto-detect serial port."""
        system = platform.system()
        
        if system == "Windows":
            return "COM5"
        elif system == "Linux":
            import os
            ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']
            for port in ports:
                if os.path.exists(port):
                    return port
            return '/dev/ttyUSB0'
        else:
            return '/dev/tty.usbserial'
    
    def _print_troubleshooting(self):
        """Print troubleshooting help."""
        system = platform.system()
        print("Troubleshooting:")
        print("  1. Check USB cable")
        print("  2. Check sensor power")
        if system == "Windows":
            print("  3. Device Manager > Ports (COM & LPT)")
        else:
            print("  3. ls /dev/tty*")
            print("  4. sudo chmod 666 /dev/ttyUSB0")
    
    def read_distance(self) -> Optional[float]:
        """
        Read one distance measurement from sensor.
        This version BLOCKS and WAITS for data!
        
        Returns:
            Distance in cm (or None if timeout/error)
        """
        try:
            # BLOCKING READ - wait for a line of data
            line = self.serial.readline()
            
            if not line:
                # Timeout - no data received
                if self.debug:
                    print("[TIMEOUT] No data received")
                return None
            
            # Decode
            line_str = None
            for encoding in ['utf-8', 'ascii', 'latin-1']:
                try:
                    line_str = line.decode(encoding).strip()
                    break
                except:
                    continue
            
            if line_str is None or len(line_str) == 0:
                if self.debug:
                    print(f"[EMPTY] Could not decode")
                return None
            
            # Debug: show raw data
            if self.debug:
                print(f"[RAW] '{line_str}'")
            
            self.total_readings += 1
            
            # Parse distance
            distance = self._parse_distance(line_str)
            
            if distance is not None:
                self.valid_readings += 1
                
                if self.debug:
                    print(f"[OK] {distance:.1f} cm\n")
                
                return distance
            else:
                self.parse_errors += 1
                if self.debug:
                    print(f"[ERROR] Could not parse: '{line_str}'\n")
                return None
            
        except Exception as e:
            if self.debug:
                print(f"[EXCEPTION] {e}")
            return None
    
    def _parse_distance(self, line: str) -> Optional[float]:
        """
        Parse distance from sensor output.
        """
        if not line:
            return None
        
        # Try direct conversion first
        try:
            distance = float(line)
            if 2.0 <= distance <= 400.0:
                return distance
        except ValueError:
            pass
        
        # Remove common text
        cleaned = line.lower()
        for word in ['distance', 'dist', 'range', 'cm', 'mm', 'm', ':', '=', 'r']:
            cleaned = cleaned.replace(word, ' ')
        
        cleaned = cleaned.strip()
        
        try:
            distance = float(cleaned)
            if 2.0 <= distance <= 400.0:
                return distance
        except ValueError:
            pass
        
        # Extract first number
        import re
        numbers = re.findall(r'\d+\.?\d*', line)
        
        if numbers:
            try:
                distance = float(numbers[0])
                if 2.0 <= distance <= 400.0:
                    return distance
            except ValueError:
                pass
        
        return None
    
    def get_statistics(self) -> dict:
        """Get sensor statistics."""
        success_rate = (self.valid_readings / self.total_readings * 100) \
                      if self.total_readings > 0 else 0
        
        return {
            'total_readings': self.total_readings,
            'valid_readings': self.valid_readings,
            'parse_errors': self.parse_errors,
            'success_rate': success_rate
        }
    
    def close(self):
        """Close serial connection."""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            print("\nSensor disconnected")