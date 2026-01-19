# Lane Keeping System Documentation

## Overview
This document describes the lane keeping system implementation for stable lane keeping on the BFMC embedded platform.

## Architecture

The lane keeping system consists of four main components:

### 1. IR Camera Driver (`drivers/ircamera`)
- **Purpose**: Interfaces with a 5-sensor IR array to detect lane lines
- **Pin Configuration**: A0, A1, A3, A5, A6 (analog inputs)
- **Features**:
  - Binary line detection (line present/absent on each sensor)
  - Weighted position calculation (-100 to +100 scale)
  - Calibration support for threshold adjustment
  - Line detection status

### 2. Lane Perception (`brain/laneperception`)
- **Purpose**: Processes IR camera data to extract stable lane position
- **Features**:
  - Real-time lane position tracking
  - Exponential moving average filter (alpha = 0.3)
  - Lane detection status management
  - Reset capability for initialization

### 3. PID Controller (`utils/pidcontroller`)
- **Purpose**: Generic PID controller for closed-loop control
- **Features**:
  - Proportional, Integral, Derivative control terms
  - Output clamping with limits
  - Anti-windup protection for integral term
  - Configurable gains (Kp, Ki, Kd)

### 4. Lane Keeping Controller (`periodics/lanekeeping`)
- **Purpose**: Main control loop integrating perception and PID
- **Control Frequency**: 50 Hz (20ms period)
- **Default PID Gains**:
  - Kp = 1.5 (Proportional gain)
  - Ki = 0.01 (Integral gain)
  - Kd = 0.5 (Derivative gain)
- **Steering Limits**: -250 to +250 (±25.0 degrees)

## Control Flow

```
IR Sensors → Camera Driver → Lane Perception → PID Controller → Steering Motor
                                    ↓
                              Speed Controller
```

1. **IR Camera** reads 5 analog sensors and detects line position
2. **Lane Perception** filters the position for smooth tracking
3. **PID Controller** calculates steering correction based on lane error
4. **Lane Keeping** commands steering motor and speed motor
5. **Feedback** sent via serial for monitoring

## Serial Commands

The lane keeping system can be controlled via serial commands:

### Enable Lane Keeping
```
lanekeeping;1;200
```
- Enables lane keeping with speed = 200 mm/s

### Disable Lane Keeping
```
lanekeeping;0
```
- Disables lane keeping and stops the vehicle

### Configure PID Gains
```
lanekeeping;pid;1.5;0.01;0.5
```
- Sets Kp=1.5, Ki=0.01, Kd=0.5

### Status Feedback
The system sends status messages via serial:
```
@lk:1;-25;-38;;     # Lane detected, error=-25, steering=-38
@lk:0;0;0;;         # Lane not detected
```

## Hardware Requirements

### IR Sensor Array
- 5 IR sensors arranged in a horizontal line
- Connected to analog pins: A0, A1, A3, A5, A6
- Sensors should be positioned to detect lane markings
- Recommended spacing: 2-3 cm between sensors

### Pin Assignments
- A0: Left-most sensor
- A1: Left-center sensor
- A3: Center sensor
- A5: Right-center sensor
- A6: Right-most sensor
- D3: Speed motor control (existing)
- D4: Steering motor control (existing)

## Tuning Guide

### PID Tuning
1. **Start with P-only control**: Set Ki=0, Kd=0
   - Increase Kp until oscillation occurs
   - Reduce Kp to 60-70% of oscillation value

2. **Add D control**: 
   - Set Kd = Kp/10
   - Adjust to dampen oscillations

3. **Add I control**:
   - Set Ki = Kp/100
   - Increase slowly to eliminate steady-state error

### Filter Tuning
- Increase `m_filter_alpha` (0.3 → 0.5) for faster response
- Decrease `m_filter_alpha` (0.3 → 0.1) for smoother tracking

### Speed Selection
- Start with low speed (100-150 mm/s) for initial testing
- Increase gradually as control stability improves
- Higher speeds may require different PID gains

## Safety Features

1. **Lane Loss Detection**: System stops when lane is not detected
2. **Output Limiting**: Steering commands are clamped to safe limits
3. **Anti-windup**: Prevents integral term saturation
4. **Emergency Stop**: Can be disabled via serial command at any time

## Integration with System

The lane keeping system is integrated into the main task scheduler:
- Runs as a periodic task at 50 Hz
- Can coexist with other control modes
- Properly initialized in main.cpp
- Serial command registered with command dispatcher

## Testing Recommendations

1. **Static Testing**:
   - Place vehicle over line
   - Enable lane keeping at 0 speed
   - Verify sensor readings and position calculation

2. **Low Speed Testing**:
   - Test at 50-100 mm/s on straight line
   - Verify smooth steering response
   - Check for oscillations

3. **Normal Operation**:
   - Gradually increase speed
   - Test on curves and straight sections
   - Monitor serial output for stability

4. **PID Tuning**:
   - Record lane error and steering response
   - Adjust gains for optimal performance
   - Document final working parameters

## Troubleshooting

### Problem: Vehicle oscillates
- **Solution**: Reduce Kp, increase Kd

### Problem: Doesn't center on line
- **Solution**: Increase Ki slightly

### Problem: Lane not detected
- **Solution**: Calibrate IR sensors, adjust threshold

### Problem: Unstable at high speed
- **Solution**: Use speed-dependent PID gains or reduce speed

## Future Enhancements

Possible improvements for better stability:
1. Adaptive PID gains based on speed
2. Kalman filter for lane position estimation
3. Multiple sensor arrays for better coverage
4. Lookahead control using predicted lane position
5. Integration with IMU for better stability
