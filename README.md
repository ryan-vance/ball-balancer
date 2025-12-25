# Ball Balancing Robot

Autonomous ball-balancing robot using a 3-RRS parallel manipulator controlled by a custom PID algorithm. Achieves real-time position tracking via resistive touchscreen and maintains ball equilibrium through dynamic platform tilting with 3 stepper motors.

## Overview

This project demonstrates the ability to work as a team to design, create, and document a project that highlights concepts from ENGR 373: Linear Control Systems. Scope, specifications, and success criteria were determined by the students and approved by the advisor.

## Project Scope & Specifications

### Requirements

Our team established three core requirements for this project:

**Functionality**
- Balance the ball at desired origin
- Gather real-time data on ball position
- Use feedback control to tilt platform accordingly

**Repeatability**
- Reliably move the ball to target positions
- Maintain consistent performance across multiple trials

**Learning Objectives**
- Gain hands-on experience with control systems
- Implement feedback controller from theory to practice
- Integrate mechanical, electrical, and software components

### Success Criteria

- ✓ Reliably move ball to desired location, regardless of initial placement
- ✓ Professional integration allowing for efficiency, functionality, and ease of use
- ✓ Fast response with minimal overshoot - no accidental ball ejection

## Evaluation

The ball-balancing robot successfully met all established project requirements, demonstrating a professional integration of hardware, software, and linear control theory. The system was evaluated based on functionality, repeatability, and construction quality.

### Key Achievement Metrics:

- **Functionality**: Real-time ball positioning with accurate touchscreen detection and responsive platform tilting
- **Repeatability**: Consistent convergence to target position regardless of initial ball placement, maintained across all startup cycles
- **Stability**: Refined PID tuning achieves rapid response times with minimal overshoot, preventing ball ejection from the platform

<img width="927" height="751" alt="image" src="https://github.com/user-attachments/assets/71b7e0d6-b77b-4da6-aadd-3fbcdc68c043" />

## Hardware Platform

This project uses the mechanical platform from [Ball-Balancer-V2](https://github.com/aaedmusa/Ball-Balancer-V2) by aaedmusa. We designed and implemented the complete control system, including:

- Custom PID controller with hand-tuned gains (Kp=0.15, Ki=0.02, Kd=0.075)
- Inverse kinematics calculations for 3-RRS manipulator
- Real-time position tracking using resistive touchscreen
- Motor control and trajectory planning algorithms

For hardware specifications, CAD files, and bill of materials, see the [original repository](https://github.com/aaedmusa/Ball-Balancer-V2).

## Our Implementation

- **Complete control system architecture** - All software and code to accomplish the course objectives
- **Miss detection logic** - Abnormal false readings on the touchscreen filtered out before being fed into the PID loop for smoother system performance
- **Movement patterns** - Line, triangle, and polygon trajectory generation
- **PID controller** - Designed, tuned, and implemented from scratch
- **Inverse kinematics calculations** - Derived and coded the mathematics for the delta RRS robot
- **System integration**
  - Touchscreen calibration
  - Motor driver tuning
  - System power requirements optimization

## Features

- Real-time ball position tracking via resistive touchscreen
- Custom PID control loop running at 50Hz
- Inverse kinematics for delta robot positioning
- Anti-windup integral protection
- Ball loss detection and recovery
- Pattern generation (line, triangle, polygon trajectories)

## How It Works

Check out the [Flowchart](https://github.com/ryan-vance/ball-balancer/blob/main/docs/ENGR%20373%20Flow%20Chart.png) to see the system process.


**Key Steps:**
1. **Detection**: Resistive touchscreen reads ball position
2. **Control**: PID algorithm calculates required platform tilt
3. **Kinematics**: Inverse kinematics converts tilt angles to motor positions
4. **Actuation**: Stepper motors move to calculated positions

For detailed implementation, see our [System Integration Documentation](docs/README.md).

## Demo

Click the link to view the [Demonstration](https://youtube.com/shorts/-RZ6nNGylSo?feature=share) of the project in action.

Additional photos and videos can be found [here]().

## PID Tuning Process

[How the system was tuned](https://github.com/ryan-vance/ball-balancer/blob/main/docs/PID%20tuning.md)

The following image shows the PID process within our system
<img width="2370" height="1218" alt="image" src="https://github.com/user-attachments/assets/4ff57d5e-0e63-434e-bcc3-6158a239e88c" />


## Team

- Noah Seys
- Hannah Stevenson  
- Maria Uribe
- Ryan Vance

## Acknowledgments

- Hardware design: [Ball-Balancer-V2](https://github.com/aaedmusa/Ball-Balancer-V2)
- Course: ENGR 373 - Linear Control Systems, Western Illinois University
- Advisor: Il-Seop Shin

## License

Our control system code is licensed under the MIT License - see LICENSE file.

Hardware design credit to original repository (linked above).
