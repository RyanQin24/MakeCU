# Raspberry Pi Setup Guide - ArUco Detection System

This guide walks you through uploading and running the ArUco detection scripts on your Raspberry Pi.

## Overview

Three scripts are provided:

1. **`aruco_detector.py`** - Original PC version (Windows/Mac/Linux)
2. **`aruco_detector_rpi.py`** - RPi-optimized standalone detector (for testing)
3. **`aruco_motor_control.py`** - Integrated motor control with ArUco detection
4. **`receiver_pi.py`** - Manual WASD control fallback (your existing script)

---

## Prerequisites

### Hardware Required

- Raspberry Pi (3B+, 4, or 5 recommended)
- USB Webcam
- Arduino/motor controller connected via USB (`/dev/ttyACM0`)
- MicroSD card with Raspberry Pi OS installed
- Power supply for Raspberry Pi

### Network Setup

- Raspberry Pi connected to same network as your PC (WiFi or Ethernet)
- Know your Raspberry Pi's IP address (find with `hostname -I` on the Pi)

---

## Step 1: Enable SSH on Raspberry Pi

If you haven't already enabled SSH:

### Option A: Using the Raspberry Pi directly (with monitor/keyboard)

```bash
sudo raspi-config
# Navigate to: Interfacing Options → SSH → Enable
```

### Option B: Enable SSH without monitor (if you have physical access to SD card)

1. Remove SD card from Pi and insert into your PC
2. Create an empty file named `ssh` (no extension) in the boot partition
3. Reinsert SD card and power on the Pi

---

## Step 2: Find Your Raspberry Pi's IP Address

From your PC, try to find the Pi on your network:

**Windows (PowerShell):**

```powershell
# Method 1: If you know the hostname (default is 'raspberrypi')
ping raspberrypi.local

# Method 2: Use arp to scan local network
arp -a | Select-String "raspberrypi"
```

**If connected directly to Pi with monitor:**

```bash
hostname -I
```

Example IP: `192.168.1.100` (use yours in commands below)

---

## Step 3: Transfer Files to Raspberry Pi

### Method A: Using SCP (Secure Copy) - Recommended

**From Windows PowerShell (in your project directory):**

```powershell
# Navigate to your project folder
cd "C:\Users\zouju\Coding Projects\MakeCU"

# Copy the ArUco scripts to Pi (replace 192.168.1.100 with your Pi's IP)
scp aruco_detector_rpi.py pi@192.168.1.100:~/
scp aruco_motor_control.py pi@192.168.1.100:~/
scp receiver_pi.py pi@192.168.1.100:~/

# You'll be prompted for password (default: raspberry)
```

### Method B: Using WinSCP (GUI tool)

1. Download WinSCP: https://winscp.net/
2. Connect to your Pi:
   - Protocol: SCP
   - Host: Your Pi's IP (e.g., 192.168.1.100)
   - Username: `pi`
   - Password: `raspberry` (or your custom password)
3. Drag and drop the three `.py` files to the Pi's home directory (`/home/pi/`)

### Method C: Using VS Code Remote-SSH Extension

1. Install "Remote - SSH" extension in VS Code
2. Press `Ctrl+Shift+P` → "Remote-SSH: Connect to Host"
3. Enter: `pi@192.168.1.100`
4. Copy files via the file explorer

---

## Step 4: SSH into Raspberry Pi

**From Windows PowerShell:**

```powershell
ssh pi@192.168.1.100
# Default password: raspberry
```

You should now see a terminal prompt on your Raspberry Pi.

---

## Step 5: Install Dependencies on Raspberry Pi

Once connected via SSH, run these commands:

```bash
# Update package lists
sudo apt-get update

# Install system packages (includes OpenCV, NumPy, and Python Serial)
sudo apt-get install -y python3-opencv python3-numpy python3-serial

# Alternative: Install via pip (if apt packages are outdated)
# pip3 install opencv-python opencv-contrib-python numpy pyserial

# Verify installations
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
python3 -c "import numpy; print('NumPy version:', numpy.__version__)"
python3 -c "import serial; print('PySerial installed successfully')"
```

**Expected output:**

```
OpenCV version: 4.x.x
NumPy version: 1.x.x
PySerial installed successfully
```

---

## Step 6: Configure Camera Permissions

Ensure your user has access to the camera:

```bash
# Add pi user to video group (usually already done)
sudo usermod -a -G video pi

# Check camera is detected
ls -l /dev/video*
# Should show: /dev/video0 (or /dev/video1, etc.)

# Test camera with simple capture
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera opened:', cap.isOpened()); cap.release()"
# Expected: Camera opened: True
```

---

## Step 7: Configure Serial Port Permissions

Ensure access to the Arduino/motor controller:

```bash
# Check serial device exists
ls -l /dev/ttyACM0
# Should show: crw-rw---- 1 root dialout ... /dev/ttyACM0

# Add pi user to dialout group
sudo usermod -a -G dialout pi

# IMPORTANT: Log out and back in for group changes to take effect
exit
# Then SSH back in:
ssh pi@192.168.1.100

# Verify group membership
groups
# Should include: dialout video
```

---

## Step 8: Test ArUco Detection (Standalone)

Test the standalone detector first (without motors):

```bash
# Run in headless mode (no display - for SSH use)
python3 ~/aruco_detector_rpi.py --headless

# OR with display (if you have monitor connected or X11 forwarding)
python3 ~/aruco_detector_rpi.py

# Test with custom camera settings
python3 ~/aruco_detector_rpi.py --headless --width 320 --height 240 --fps 15
```

**Expected output:**

```
INFO: Camera initialized: 640x480
INFO: Headless mode: True
INFO: Marker ID 22 detected at center: (320, 240)
...
```

Press `Ctrl+C` to stop.

---

## Step 9: Test Motor Control Integration

Now test the integrated motor control script:

```bash
# Run with default settings
python3 ~/aruco_motor_control.py --headless

# Run with debug logging
python3 ~/aruco_motor_control.py --headless --debug

# Run with custom settings
python3 ~/aruco_motor_control.py --headless --width 320 --height 240 --serial /dev/ttyACM0 --baud 57600
```

**What to expect:**

- Script starts, motors stop initially
- When ArUco marker is detected:
  - If marker is **left of center** → robot turns left
  - If marker is **right of center** → robot turns right
  - If marker is **centered** → robot moves forward (if marker is large enough/close)
- When no marker is detected → motors stop

**Expected output:**

```
INFO: Serial connection established: /dev/ttyACM0 @ 57600 baud
INFO: Camera initialized: 640x480
INFO: Headless mode: True
INFO: Center tolerance: ±50px, Min marker area: 1000px²
INFO: Marker ID 22 | Center: (300, 240) | Area: 2500 | Action: Turn Right
INFO: Marker ID 22 | Center: (320, 240) | Area: 2800 | Action: Forward (centered)
INFO: No marker detected - motors stopped
...
```

Press `Ctrl+C` to stop safely (motors will stop automatically).

---

## Step 10: Run Manual WASD Control (Fallback)

Keep your existing manual control as a fallback:

```bash
# In a separate SSH session or terminal
python3 ~/receiver_pi.py
```

From your PC, use your WASD sender script to control manually.

---

## Step 11: Run Scripts Automatically on Boot (Optional)

To start scripts automatically when Pi boots:

### Create systemd service for motor control

```bash
# Create service file
sudo nano /etc/systemd/system/aruco-motor.service
```

**Paste this content:**

```ini
[Unit]
Description=ArUco Motor Control Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStart=/usr/bin/python3 /home/pi/aruco_motor_control.py --headless
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Enable and start the service:**

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable service (start on boot)
sudo systemctl enable aruco-motor.service

# Start service now
sudo systemctl start aruco-motor.service

# Check status
sudo systemctl status aruco-motor.service

# View logs
journalctl -u aruco-motor.service -f
```

**To stop the service:**

```bash
sudo systemctl stop aruco-motor.service
```

---

## Tuning and Configuration

### Adjust Center Tolerance

Edit `aruco_motor_control.py` and change:

```python
CENTER_TOLERANCE = 50  # pixels - increase for looser centering
```

### Adjust Minimum Marker Area

```python
MIN_MARKER_AREA = 1000  # pixels² - decrease if robot won't move forward
```

### Adjust Turn Speed

```python
TURN_SPEED_DIVISOR = 2  # increase for slower turns (smoother)
```

### Change Camera Resolution (for better performance)

```bash
python3 ~/aruco_motor_control.py --headless --width 320 --height 240
```

---

## Troubleshooting

### Camera not opening

```bash
# Check camera is detected
ls /dev/video*
# Try different camera index: --camera 1

# Check permissions
groups  # Should include 'video'

# Test with simple script
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"
```

### Serial port errors

```bash
# Check device exists
ls -l /dev/ttyACM*
# May be /dev/ttyACM2 or /dev/ttyUSB0 - adjust --serial parameter

# Check permissions
groups  # Should include 'dialout'

# Log out and back in after adding to dialout group
exit
ssh pi@192.168.1.100
```

### ArUco markers not detected

- Ensure good lighting
- Print markers larger (at least 5cm x 5cm)
- Ensure marker is flat and not wrinkled
- Use marker ID 0-49 from DICT_4X4_50
- Generate markers: https://chev.me/arucogen/ (select 4x4, ID 0-49)

### Robot moves erratically

- Reduce `TURN_SPEED_DIVISOR` for slower, smoother turns
- Increase `CENTER_TOLERANCE` for less sensitive centering
- Adjust `MIN_MARKER_AREA` threshold

### ImportError or ModuleNotFoundError

```bash
# Reinstall packages
pip3 install --upgrade opencv-python opencv-contrib-python numpy pyserial

# OR use system packages
sudo apt-get install --reinstall python3-opencv python3-numpy python3-serial
```

---

## Testing Checklist

- [ ] SSH connection to Pi working
- [ ] Files transferred successfully
- [ ] Python packages installed
- [ ] Camera detected (`/dev/video0`)
- [ ] Serial device detected (`/dev/ttyACM0`)
- [ ] User in `video` and `dialout` groups
- [ ] Standalone ArUco detector works
- [ ] Motor control script runs without errors
- [ ] Robot responds to marker position
- [ ] Manual WASD fallback works
- [ ] Scripts can run on boot (optional)

---

## Next Steps

1. **Integrate Ultrasonic Sensor**: Add distance sensing to stop at target distance
2. **Camera Calibration**: Generate camera matrix and distortion coefficients for accurate pose estimation
3. **Multiple Markers**: Extend script to handle multiple markers with different behaviors
4. **Logging**: Add CSV logging of marker positions and actions
5. **Web Interface**: Create a simple web dashboard to monitor robot status

---

## Quick Reference Commands

```bash
# SSH into Pi
ssh pi@192.168.1.100

# Copy files to Pi
scp *.py pi@192.168.1.100:~/

# Run standalone detector
python3 ~/aruco_detector_rpi.py --headless

# Run motor control
python3 ~/aruco_motor_control.py --headless

# Run manual control
python3 ~/receiver_pi.py

# View logs
tail -f /var/log/syslog | grep python

# Stop all Python scripts
pkill -f python3
```

---

## Support

If you encounter issues:

1. Check the troubleshooting section above
2. Verify all prerequisites are met
3. Test each component individually (camera, serial, detection)
4. Check logs with `journalctl -u aruco-motor.service -f`

Good luck with your ArUco detection system! 🚀
