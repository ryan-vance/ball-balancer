# Ball Balancing Robot

Autonomous ball-balancing robot using a 3-RRS parallel manipulator controlled by a custom PID algorithm. Achieves real-time position tracking via resistive touchscreen and maintains ball equilibrium through dynamic platform tilting with 3 stepper motors.

## Overview

This project demonstrates the ability to work as a team to design, create, and document a project that highlights concepts from ENGR 373: Linear Control Systems. Scope, specifications, and success criteria were determined by the students and approved by the advisor.

## Evaluation

The ball-balancing robot successfully met all established project requirements, demonstrating a professional integration of hardware, software, and linear control theory. The system was evaluated based on functionality, repeatability, and construction quality.

### Key Achievement Metrics:

- **Functionality**: Real-time ball positioning with accurate touchscreen detection and responsive platform tilting
- **Repeatability**: Consistent convergence to target position regardless of initial ball placement, maintained across all startup cycles
- **Stability**: Refined PID tuning achieves rapid response times with minimal overshoot, preventing ball ejection from the platform

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

[Code architecture explanation](https://github.com/ryan-vance/ball-balancer/tree/main/docs)

## Demo

[Add photos and videos here]

## PID Tuning Process

[How the system was tuned](https://github.com/ryan-vance/ball-balancer/blob/main/docs/PID%20tuning.md)

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
