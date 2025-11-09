# Blue Bin Motor Control with Obstacle Avoidance

## Overview

This script combines blue bin tracking (via color detection) with obstacle avoidance using triple ultrasonic sensors.

## Hardware Setup

### Serial Ports

- **Motors**: `/dev/ttyACM2` @ 57600 baud
- **Ultrasonic Sensors**: `/dev/ttyACM0` @ 9600 baud

### Sensor Configuration

Three ultrasonic sensors at 45° angles:

- **Front**: Faces forward
- **Left**: 45° front-left angle
- **Right**: 45° front-right angle

### Sensor Data Format

Serial output: `"front,left,right\n"` (comma-separated distances in cm)
Example: `"100,150,200\n"` = Front: 100cm, Left: 150cm, Right: 200cm

## Obstacle Avoidance Logic

### Distance Thresholds

- **Danger Zone**: < 10cm (stop and navigate)
- **Slow Zone**: < 30cm (reduce speed)
- **Side Warning**: < 15cm (bias away from obstacle)

### Behavior Cases

#### Case 1: Front Obstacle (< 10cm)

- Turn towards clearer side
- If left < right → Turn RIGHT
- If right < left → Turn LEFT
- If equal → Turn RIGHT (default)

#### Case 2: Slow Zone (10-30cm front)

- Continue tracking bin
- Reduce speed to 50%

#### Case 3: Side Obstacle (< 15cm)

- Reduce motor power on that side (70%)
- Bias away from obstacle while tracking

#### Case 4: All Blocked

- Front < 10cm AND Left < 15cm AND Right < 15cm
- Backup for 1 second
- Turn right for 0.5 seconds
- Re-assess situation

#### Case 5: Clear Path (> 30cm all)

- Normal bin tracking at full speed

## Usage

### Basic Usage

```bash
python3 blue_bin_motor_control.py
```

### Headless Mode (SSH/Remote)

```bash
python3 blue_bin_motor_control.py --headless
```

### Debug Mode

```bash
python3 blue_bin_motor_control.py --debug
```

### Custom Ports

```bash
python3 blue_bin_motor_control.py \
  --motor-serial /dev/ttyACM2 \
  --sensor-serial /dev/ttyACM0 \
  --motor-baud 57600 \
  --sensor-baud 9600
```

### All Options

```bash
python3 blue_bin_motor_control.py --help
```

## Command-Line Arguments

| Argument          | Default      | Description                |
| ----------------- | ------------ | -------------------------- |
| `--headless`      | False        | Run without display window |
| `--camera`        | 0            | Camera index               |
| `--motor-serial`  | /dev/ttyACM2 | Motor controller port      |
| `--sensor-serial` | /dev/ttyACM0 | Ultrasonic sensor port     |
| `--motor-baud`    | 57600        | Motor baud rate            |
| `--sensor-baud`   | 9600         | Sensor baud rate           |
| `--debug`         | False        | Enable debug logging       |

## Testing Sensors Only

To test the ultrasonic sensors independently:

```bash
python3 triple_ultrasonic_sensor.py --debug
```

## Files

- **blue_bin_motor_control.py**: Main script with obstacle avoidance
- **triple_ultrasonic_sensor.py**: Sensor module (shared)
- **blue_bin_detector.py**: Blue bin color detection
- **config_vision.py**: Configuration settings

## Motor Commands

Commands sent to Arduino in format: `"left,right>"`

Examples:

- Forward: `"260,-260>"`
- Turn Left: `"-130,130>"`
- Turn Right: `"130,-130>"`
- Stop: `"0,0>"`
- Backup: `"-200,-200>"`

## Troubleshooting

### Motors not responding

- Check `/dev/ttyACM2` connection
- Verify baud rate (57600)
- Test with: `ls -l /dev/ttyACM*`

### Sensors not reading

- Check `/dev/ttyACM0` connection
- Verify baud rate (9600)
- Test with: `python3 triple_ultrasonic_sensor.py --debug`

### Permission errors

```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM2
# Or add user to dialout group:
sudo usermod -a -G dialout $USER
```

### Camera not found

```bash
ls /dev/video*
# Try different camera index: --camera 1
```

## Safety Notes

- Always test in open space first
- Keep emergency stop ready
- Monitor sensor readings with `--debug`
- Backup duration is 1 second (adjustable in code)
- Robot will stop if no bin detected

## Performance

- Camera: ~20 FPS
- Sensor readings: ~10 Hz
- Response time: ~50-100ms
- Obstacle detection: Continuous during operation
