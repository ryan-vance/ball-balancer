# PID Controller Tuning Process

## Overview

The ball balancer uses a PID (Proportional-Integral-Derivative) controller to maintain the ball at the center of the platform. The controller continuously calculates an error value as the difference between the desired setpoint (center) and the measured ball position, then applies corrections based on proportional, integral, and derivative terms.

## Control Loop Architecture

The PID controller operates as a **negative feedback control loop**:

1. **Input**: Ball position from resistive touchscreen (x, y coordinates)
2. **Setpoint**: Center of platform (0, 0)
3. **Error**: Distance and direction from center to ball position
4. **Output**: Platform tilt angle to move ball toward center

## Tuning Method: Ziegler-Nichols

We used the **Ziegler-Nichols tuning method** to determine optimal PID gains. This is a standard heuristic method for tuning PID controllers based on system response characteristics.

### Tuning Process

1. **Started with P-only control** (Ki = 0, Kd = 0)
   - Gradually increased Kp until the system showed sustained oscillations
   - Observed system behavior and response time

2. **Added Integral term**
   - Introduced Ki to eliminate steady-state error
   - Tuned to prevent integral windup (excessive accumulation)
   - Implemented anti-windup protection with max integral limit

3. **Added Derivative term**
   - Introduced Kd to reduce overshoot and improve stability
   - Fine-tuned to dampen oscillations without introducing noise sensitivity

4. **Iterative refinement**
   - Tested with actual hardware
   - Adjusted gains based on observed performance
   - Balanced response time vs. stability

## Final PID Gains

After extensive testing and tuning, we arrived at these optimal values:

```c
float kp = 0.15;  // Proportional gain
float ki = 0.02;  // Integral gain  
float kd = 0.075; // Derivative gain
```

### Why These Values?

- **Kp = 0.15**: Provides strong corrective action without causing excessive oscillation
- **Ki = 0.02**: Small enough to prevent windup, large enough to eliminate steady-state error
- **Kd = 0.075**: Half of Kp value provides good damping without being overly sensitive to noise

## Implementation Details

### Control Loop Timing
- **Loop frequency**: 50 Hz (20ms per cycle)
- **Consistent timing**: Ensures derivative calculations remain accurate

```c
// Ensure consistent loop timing (20ms cycle)
while (millis() - startTime < 20) {
    moveToPosition(defaultHeight, -output[0], -output[1]);
}
```

### PID Calculation

The controller calculates output for X and Y axes independently:

```c
for (int i = 0; i < 2; i++) {  // For x and y dimensions
    // Calculate error
    lastError[i] = error[i];
    error[i] = target[i] - position[i];
    
    // Integral term with anti-windup
    integral[i] = constrain(integral[i] + error[i] * deltaTime, 
                           -maxIntegral, maxIntegral);
    
    // Derivative term
    derivative[i] = (error[i] - lastError[i]) / deltaTime;
    
    // PID output
    output[i] = kp * error[i] + ki * integral[i] + kd * derivative[i];
    
    // Constrain output
    output[i] = constrain(output[i], -0.25, 0.25);
}
```

## Safety Features

### 1. Integral Anti-Windup
Prevents integral term from accumulating excessively when the ball is far from center:

```c
const float maxIntegral = 175;  // Maximum integral accumulation
integral[i] = constrain(integral[i] + error[i] * deltaTime, 
                       -maxIntegral, maxIntegral);
```

### 2. Output Limiting
Constrains platform tilt to safe, achievable angles:

```c
output[i] = constrain(output[i], -0.25, 0.25);
```

### 3. Derivative Error Checking
Handles edge cases where derivative calculation might fail:

```c
// Check for NaN or Inf in derivative
if (isnan(derivative[i]) || isinf(derivative[i])) {
    derivative[i] = 0;
}
```

### 4. Ball Loss Detection
Resets PID terms when ball is no longer detected:

```c
if (missCount == 4) {
    Serial.println("Ball lost, resetting PID");
    for (int i = 0; i < 2; i++) {
        integral[i] = 0;
        derivative[i] = 0;
        output[i] = 0;
    }
    ballDetected = false;
}
```

## Performance Characteristics

With the final tuned gains, the system achieves:

- **Fast response time**: Ball reaches center within 1-2 seconds
- **Minimal overshoot**: Less than 10% overshoot on step response
- **Stable equilibrium**: Ball maintains position at center with minimal oscillation
- **Robust to disturbances**: Quickly recovers when ball is manually displaced

## Tuning Tips for Future Development

If re-tuning is needed:

1. **Increase Kp** if response is too slow
2. **Decrease Kp** if system oscillates excessively
3. **Increase Ki** if steady-state error persists
4. **Decrease Ki** if integral windup occurs
5. **Increase Kd** to reduce overshoot
6. **Decrease Kd** if system is too sensitive to noise

## Testing Methodology

We validated PID performance through:

1. **Step response tests**: Placing ball at various positions, observing return to center
2. **Disturbance rejection**: Manually moving ball and observing recovery
3. **Pattern following**: Programming specific trajectories (lines, triangles, etc.)
4. **Long-duration stability**: Running system for extended periods to verify stability

## References

- Course: ENGR 373 - Linear Control Systems, Western Illinois University
- Tuning Method: Ziegler-Nichols heuristic tuning method
- Team: Noah Seys, Hannah Stevenson, Maria Uribe, Ryan Vance
