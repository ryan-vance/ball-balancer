# System Architecture

## Overview

The ball balancer is a complex mechatronic system that integrates mechanical hardware, electronic sensors, stepper motors, and control algorithms. This document explains how all components work together to achieve stable ball balancing.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Ball Balancing System                    │
│                                                             │
│  ┌──────────────┐      ┌──────────────┐      ┌───────────┐  │
│  │  Touchscreen │────▶│     Teensy    |────▶│   Motors  │  │
│  │    Sensor    │      │      4.1     |      │ + Drivers │  │
│  │  (Position)  │      │              │      │           │  │
│  └──────────────┘      └──────────────┘      └───────────┘  │
│         │                      │                     │      │
│         │                      ▼                     │      │
│         │              ┌──────────────┐              │      │
│         │              │ PID Control  │              │      │
│         │              │   Algorithm  │              │      │
│         │              └──────────────┘              │      │
│         │                      │                     │      │
│         │                      ▼                     │      │
│         │              ┌──────────────┐              │      │
│         │              │   Inverse    │              │      │
│         │              │  Kinematics  │              │      │
│         │              └──────────────┘              │      │
│         │                                            │      │
│         └────────────────────┬───────────────────────┘      │
│                              ▼                              │
│                    ┌──────────────────┐                     │
│                    │  Delta Platform  │                     │
│                    │   (Mechanical)   │                     │
│                    └──────────────────┘                     │
└─────────────────────────────────────────────────────────────┘
```

## Software Architecture

### Main Control Loop

The system operates in a continuous control loop at 50Hz (20ms cycle time):

```c
void loop() {
    moveToTarget();
}
```

### moveToTarget() Function Flow

This is the heart of the system integration:

```
1. Read touchscreen
   ↓
2. Check if ball detected
   ↓
3a. Ball detected:               3b. Ball not detected:
    - Map coordinates to mm          - Increment miss counter
    - Calculate PID error            - If 4+ misses: reset PID
    - Compute PID output             - Set ballDetected = false
    - Constrain output               
    ↓                                ↓
4. Call moveToPosition(height, nx, ny)
   ↓
5. Calculate motor angles (inverse kinematics)
   ↓
6. Convert angles to motor steps
   ↓
7a. Ball detected:               7b. Ball not detected:
    - Dynamic speed based on         - Fixed speed (800 steps/s)
      distance to target             - Fixed accel (4000 steps/s²)
    - Filter speed changes           - Move to home position
    - Move motors                    
    ↓                                ↓
8. Run motors (non-blocking)
   ↓
9. Wait until 20ms cycle complete
   ↓
10. Repeat
```

## Data Flow

### Position Sensing
```c
// 1. Read raw touchscreen data
TSPoint point = ts.getPoint();

// 2. Center coordinates (offset from middle)
point = offsetPoint(point, 500, 500);

// 3. Map to physical coordinates (-1.0 to 1.0)
position[0] = map(point.x, -500, 500, -100, 100) / 100.0;  // X
position[1] = map(point.y, -500, 500, -100, 100) / 100.0;  // Y
```

### PID Control
```c
// Calculate error
error[i] = target[i] - position[i];

// Update integral with anti-windup
integral[i] = constrain(integral[i] + error[i] * deltaTime, 
                       -maxIntegral, maxIntegral);

// Calculate derivative
derivative[i] = (error[i] - lastError[i]) / deltaTime;

// Compute output
output[i] = kp * error[i] + ki * integral[i] + kd * derivative[i];

// Safety constraint
output[i] = constrain(output[i], -0.25, 0.25);
```

### Motor Control
```c
// For each motor (i = 0, 1, 2)
for (int i = 0; i < 3; i++) {
    // 1. Calculate required angle via inverse kinematics
    double angle = MtrAng(i, h, nx, ny);
    
    // 2. Convert to motor steps
    motorPositions[i] = round((angOrig - angle) * angToStep);
    
    // 3. Safety limit
    motorPositions[i] = constrain(motorPositions[i], 15, 550);
    
    // 4. Dynamic speed calculation (if ball detected)
    float distance = abs(steppers[i].currentPosition() - motorPositions[i]);
    motorSpeed[i] = distance * speedKs;  // speedKs = 25
    
    // 5. Speed filtering (prevent jerky motion)
    motorSpeed[i] = constrain(motorSpeed[i], 
                             prevMotorSpeed[i] - 200, 
                             prevMotorSpeed[i] + 200);
    motorSpeed[i] = constrain(motorSpeed[i], 0, 2000);
    
    // 6. Set motor parameters
    steppers[i].setMaxSpeed(motorSpeed[i]);
    steppers[i].setAcceleration(motorSpeed[i] * 30);
    
    // 7. Command motor to move
    steppers[i].moveTo(motorPositions[i]);
}

// 8. Execute one step for each motor (non-blocking)
steppers[0].run();
steppers[1].run();
steppers[2].run();
```

## Timing and Synchronization

### Control Loop Timing

Maintaining consistent 20ms loop timing is critical for PID stability:

```c
unsigned long startTime = millis();

// ... perform all calculations and motor movements ...

// Wait for remainder of 20ms cycle
while (millis() - startTime < 20) {
    // Keep motors moving while waiting
    moveToPosition(defaultHeight, -output[0], -output[1]);
}
```

**Why 20ms (50Hz)**
- Fast enough to respond to ball movement
- Slow enough for reliable touchscreen readings
- Matches common control system frequencies
- Allows time for all calculations

### Delta Time Calculation

For accurate PID derivative and integral calculations:

```c
unsigned long currentTime = millis();
float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds
previousTime = currentTime;
```

## Initialization Sequence

The `setup()` function establishes initial system state:

```c
void setup() {
    // 1. Initialize serial communication
    Serial.begin(9600);
    
    // 2. Configure motor pins as outputs
    for (int i = 0; i < 3; i++) {
        pinMode(MOTOR_PINS[i][0], OUTPUT);  // Step pin
        pinMode(MOTOR_PINS[i][1], OUTPUT);  // Direction pin
        
        // Set motor parameters
        steppers[i].setPinsInverted(true, false, false);
        steppers[i].setMaxSpeed(6000);
        steppers[i].setAcceleration(6000);
        steppers[i].setCurrentPosition(0);
    }
    
    // 3. Enable motors (active low)
    pinMode(en_pin, OUTPUT);
    digitalWrite(en_pin, HIGH);  // Disabled
    delay(500);
    digitalWrite(en_pin, LOW);   // Enabled
    
    // 4. Move to home position
    moveToPosition(defaultHeight, 0, 0);
    
    // 5. Wait until home position reached
    bool running = true;
    while (running) {
        running = false;
        for (int i = 0; i < 3; i++) {
            if (steppers[i].distanceToGo() != 0) {
                steppers[i].run();
                running = true;
            }
        }
    }
    
    // 6. Initialize timing
    previousTime = millis();
}
```

## Error Handling and Safety

### Ball Detection Safety

Prevents false positives and handles ball loss gracefully:

```c
if (point.x != 0) {
    // Ball detected - reset miss counter
    missCount = 0;
    ballDetected = true;
} else {
    // Possible miss - double check
    point = ts.getPoint();
    if (point.x == 0) {
        missCount++;
        if (missCount == 4) {
            // Ball definitely lost - reset system
            Serial.println("Ball lost, resetting PID");
            for (int i = 0; i < 2; i++) {
                integral[i] = 0;
                derivative[i] = 0;
                output[i] = 0;
            }
            ballDetected = false;
        }
    }
}
```

### Derivative Error Protection

Prevents invalid calculations from crashing the system:

```c
// Check for NaN (Not a Number) or Inf (Infinity)
if (isnan(derivative[i]) || isinf(derivative[i])) {
    derivative[i] = 0;
}
```

This can occur when:
- deltaTime is zero or very small
- Division by zero
- Sensor noise causes extreme values

### Motor Position Constraints

Physical safety limits prevent damage to hardware:

```c
motorPositions[i] = constrain(motorPositions[i], 15, 550);
```

These limits ensure:
- Motors don't stall at mechanical stops
- Arms don't over-extend
- Platform stays in safe operating range

## Libraries Used

### TouchScreen Library
- **Source**: Adafruit TouchScreen library
- **Purpose**: Simplifies resistive touchscreen reading
- **Functions used**:
  - `TouchScreen(xp, yp, xm, ym, resistance)`
  - `getPoint()` - Read X/Y coordinates

### AccelStepper Library  
- **Purpose**: Advanced stepper motor control
- **Features**:
  - Acceleration/deceleration control
  - Non-blocking movement
  - Multiple motor coordination
- **Functions used**:
  - `setMaxSpeed()`, `setAcceleration()`
  - `moveTo()`, `run()`
  - `currentPosition()`, `distanceToGo()`

## Calibration and Tuning

### Touchscreen Calibration
```c
// Offset values determined experimentally
point = offsetPoint(point, 500, 500);

// Mapping verified by placing ball at known positions
position[0] = map(point.x, -500, 500, -100, 100) / 100.0;
```

### Motor Direction
```c
// Ensure motors rotate in correct direction
steppers[i].setPinsInverted(true, false, false);
```

### Home Position Angle
```c
// Starting angle experimentally determined
double angOrig = 206.662752199;
```

This angle represents the motor position when platform is:
- At default height (4.25 inches)
- Perfectly horizontal (no tilt)

## Performance Optimization

### Non-Blocking Motor Control
Using `AccelStepper.run()` instead of blocking commands allows:
- Simultaneous multi-motor operation
- Continued sensor reading during motion
- Smooth, coordinated movement

### Speed Adaptation
```c
// Dynamic speed based on distance to target
float distance = abs(steppers[i].currentPosition() - motorPositions[i]);
motorSpeed[i] = distance * speedKs;

// Smooth speed changes
motorSpeed[i] = constrain(motorSpeed[i], 
                         prevMotorSpeed[i] - 200, 
                         prevMotorSpeed[i] + 200);
```

Benefits:
- Fast movement when far from target
- Slow, precise movement near target
- No sudden jerks or vibrations

## Advanced Features

### Pattern Generation

The system can balance the ball while following programmed patterns:

```c
// Line pattern
patternLine(x1, y1, x2, y2, wait_time, repetitions);

// Triangle pattern  
patternTriangle(x1, y1, x2, y2, x3, y3, wait_time, repetitions);

// General polygon pattern
patternPolygon(coords_array, wait_time, repetitions);
```

These functions:
- Set target position to pattern waypoints
- Wait specified time for ball to reach each point
- Repeat pattern multiple times
- Demonstrate control system accuracy

## Troubleshooting Integration Issues

### Ball Not Detected
- Check touchscreen connections
- Verify ball weight is sufficient
- Check offset values in code

### Motors Not Moving
- Verify enable pin is LOW (enabled)
- Check power supply voltage
- Confirm motor driver connections

### Platform Oscillating
- PID gains too high (especially Kp)
- Reduce Kp or increase Kd
- See PID tuning documentation

### Inconsistent Movement
- Check loop timing (should be consistent 20ms)
- Verify motor speed constraints
- Check for mechanical binding

## Future Integration Improvements

Potential enhancements to the system:

1. **Sensor fusion**: Add IMU for direct platform angle measurement
2. **Adaptive tuning**: Automatically adjust PID gains based on performance
3. **Visual feedback**: LED indicators for system state
4. **Wireless control**: Bluetooth/WiFi for parameter adjustment
5. **Data logging**: SD card storage for performance analysis

## Team and Acknowledgments

- **Hardware platform**: [Ball-Balancer-V2](https://github.com/aaedmusa/Ball-Balancer-V2)
- **Control system implementation**: Our team
- **Course**: ENGR 373 - Linear Control Systems, Western Illinois University
- **Team**: Noah Seys, Hannah Stevenson, Maria Uribe, Ryan Vance
