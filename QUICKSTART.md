# ArUco Detection System - Quick Start Guide

## 📁 Files Overview

### Core Scripts

- **`aruco_detector.py`** - Original Windows/PC version with GUI
- **`aruco_detector_rpi.py`** - Raspberry Pi optimized standalone detector
- **`aruco_motor_control.py`** - ⭐ Main integration script (ArUco + Motors)
- **`receiver_pi.py`** - Manual WASD control fallback

### Documentation

- **`README_RPI_SETUP.md`** - Complete step-by-step setup guide
- **`requirements.txt`** - Python dependencies

---

## 🚀 Quick Start (30 seconds)

### 1. Transfer files to Raspberry Pi

```powershell
# From your project folder on Windows:
cd "C:\Users\zouju\Coding Projects\MakeCU"
scp aruco_motor_control.py requirements.txt pi@YOUR_PI_IP:~/
```

### 2. SSH into Pi and install dependencies

```bash
ssh pi@YOUR_PI_IP
sudo apt-get update
sudo apt-get install -y python3-opencv python3-numpy python3-serial
```

### 3. Run the motor control script

```bash
python3 ~/aruco_motor_control.py --headless
```

That's it! Your robot will now:

- ✅ Detect ArUco markers (ID 0-49, DICT_4X4_50)
- ✅ Turn left/right to center the marker
- ✅ Move forward when marker is centered
- ✅ Stop when no marker is detected

Press `Ctrl+C` to stop.

---

## 🎯 How It Works

### Detection Logic

1. Camera captures frame (640x480, 30 FPS)
2. Convert to grayscale and detect ArUco markers
3. Calculate marker center position `(cx, cy)`
4. Compare to frame center

### Movement Logic

```
If marker detected:
    If marker is LEFT of center (cx < center - 50px):
        → Turn LEFT

    If marker is RIGHT of center (cx > center + 50px):
        → Turn RIGHT

    If marker is CENTERED (within ±50px):
        If marker is large enough (area > 1000px²):
            → Move FORWARD
        Else:
            → STOP (too far)
Else (no marker):
    → STOP motors
```

### Motor Commands (Serial)

- Format: `"left_power,right_power>"` (e.g., `"260,-260>"`)
- Serial port: `/dev/ttyACM0` @ 57600 baud
- Forward: `260,-260`
- Turn Left: `-130,-130`
- Turn Right: `130,130`
- Stop: `0,0`

---

## ⚙️ Configuration

### Common Adjustments

Edit `aruco_motor_control.py` to tune behavior:

```python
# Line 37-41 - Motor Power Values
FORWARD_LEFT_POWER = 260      # Increase for faster forward
FORWARD_RIGHT_POWER = -260
LEFT_L_POWER = 260             # Adjust for turn speed
LEFT_R_POWER = 260

# Line 48 - Center Tolerance
CENTER_TOLERANCE = 50          # ±pixels - increase for looser centering

# Line 49 - Turn Speed
TURN_SPEED_DIVISOR = 2         # Increase for slower, smoother turns

# Line 50 - Minimum Marker Area
MIN_MARKER_AREA = 1000         # Decrease if robot won't move forward
```

### Command-Line Options

```bash
# Run with default settings
python3 aruco_motor_control.py --headless

# Low resolution for better performance
python3 aruco_motor_control.py --headless --width 320 --height 240

# Debug mode (verbose logging)
python3 aruco_motor_control.py --headless --debug

# Custom serial port
python3 aruco_motor_control.py --headless --serial /dev/ttyUSB0

# With display (if monitor connected)
python3 aruco_motor_control.py
```

---

## 🔧 Troubleshooting

### Camera Issues

```bash
# Check camera device
ls /dev/video*

# Test camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('OK' if cap.isOpened() else 'FAIL')"

# Try different camera index
python3 aruco_motor_control.py --headless --camera 1
```

### Serial Issues

```bash
# Check serial device
ls -l /dev/ttyACM*

# Fix permissions
sudo usermod -a -G dialout pi
exit  # Log out and back in

# Try different serial port
python3 aruco_motor_control.py --headless --serial /dev/ttyUSB0
```

### Markers Not Detected

- ✅ Use good lighting (avoid shadows)
- ✅ Print markers at least 5cm x 5cm
- ✅ Keep markers flat (no wrinkles)
- ✅ Use IDs 0-49 for DICT_4X4_50
- ✅ Generate markers: https://chev.me/arucogen/

### Robot Behavior Issues

**Robot won't move forward:**

- Decrease `MIN_MARKER_AREA` (line 50)
- Move marker closer to camera
- Check motor power values are correct

**Turns too fast:**

- Increase `TURN_SPEED_DIVISOR` (line 49)
- Reduce `LEFT_L_POWER` and `LEFT_R_POWER`

**Too sensitive to centering:**

- Increase `CENTER_TOLERANCE` (line 48)

---

## 📊 Testing Checklist

Before deployment:

- [ ] SSH works: `ssh pi@YOUR_PI_IP`
- [ ] Files transferred: `ls ~/*.py`
- [ ] Dependencies installed: `python3 -c "import cv2, numpy, serial"`
- [ ] Camera works: `/dev/video0` exists
- [ ] Serial works: `/dev/ttyACM0` exists
- [ ] Groups correct: `groups` shows `video dialout`
- [ ] Standalone detector works: `python3 aruco_detector_rpi.py --headless`
- [ ] Motor control works: `python3 aruco_motor_control.py --headless`

---

## 🎓 Next Steps

### 1. Generate ArUco Markers

Visit: https://chev.me/arucogen/

- Dictionary: 4x4 (50, 100, 250, 1000)
- Marker ID: 0-49
- Marker size: 200mm
- Download and print

### 2. Camera Calibration (Optional)

For accurate distance/pose estimation:

```bash
# Install calibration tools
pip3 install opencv-contrib-python

# Run calibration (print chessboard pattern first)
python3 camera_calibration.py
```

### 3. Add Ultrasonic Sensor

Integrate distance sensor to stop at target distance (you mentioned this will come later).

### 4. Multi-Marker Support

Extend to handle multiple markers with different IDs and behaviors.

---

## 📱 Remote Monitoring

### View logs in real-time

```bash
# SSH into Pi
ssh pi@YOUR_PI_IP

# Run with debug output
python3 aruco_motor_control.py --headless --debug

# OR tail system logs
tail -f /var/log/syslog | grep python
```

### Stop all scripts

```bash
pkill -f python3
```

---

## 🛠️ Auto-Start on Boot

See **Step 11** in `README_RPI_SETUP.md` for systemd service setup.

Quick version:

```bash
sudo nano /etc/systemd/system/aruco-motor.service
# (Paste service configuration from README_RPI_SETUP.md)

sudo systemctl enable aruco-motor.service
sudo systemctl start aruco-motor.service
```

---

## 📞 Support

For detailed instructions, see: **`README_RPI_SETUP.md`**

Common issues:

1. Camera not detected → Check `/dev/video*` and permissions
2. Serial errors → Check `/dev/ttyACM*` and dialout group
3. Markers not detected → Check lighting and marker size
4. Robot behavior → Adjust tolerance and speed constants

---

## 🎉 You're All Set!

Your ArUco detection system is ready. The robot will:

- Automatically detect and center ArUco markers
- Move forward when centered
- Stop when no marker is visible
- Fall back to manual WASD control when needed

Happy coding! 🤖
