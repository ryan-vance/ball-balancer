# Ball Balancing Robot

Autonomous ball-balancing robot using a 3-RRS parallel manipulator controlled by a custom PID algorithm. Achieves real-time position tracking via resistive touchscreen and maintains ball equilibrium through dynamic platform tilting with 3 stepper motors.

## Overview
[Your description of what the project does]

## Hardware Platform
This project uses the mechanical platform from [Ball-Balancer-V2](https://github.com/aaedmusa/Ball-Balancer-V2) by aaedmusa. We designed and implemented the complete control system, including:

- Custom PID controller with hand-tuned gains (Kp=0.15, Ki=0.02, Kd=0.075)
- Inverse kinematics calculations for 3-RRS manipulator
- Real-time position tracking using resistive touchscreen
- Motor control and trajectory planning algorithms

For hardware specifications, CAD files, and bill of materials, see the [original repository](https://github.com/aaedmusa/Ball-Balancer-V2).

## Our Implementation
[What YOUR team actually built]

## Features
- Real-time ball position tracking via resistive touchscreen
- Custom PID control loop running at 50Hz
- Inverse kinematics for delta robot positioning
- Anti-windup integral protection
- Ball loss detection and recovery
- Pattern generation (line, triangle, polygon trajectories)

## How It Works
[Your system architecture explanation]

## Demo
[YOUR photos and videos]

## Code Architecture
[Explanation of YOUR code structure]

## PID Tuning Process
[How YOU tuned the system]

## Team
- Noah Seys
- Hannah Stevenson  
- Maria Uribe
- Ryan Vance

## Acknowledgments
- Hardware design: [Ball-Balancer-V2](https://github.com/aaedmusa/Ball-Balancer-V2)
- Course: ENGR 373 - Linear Control Systems, Western Illinois University
- Advisor: [Professor name if applicable]

## License
Our control system code is licensed under the MIT License - see LICENSE file.

Hardware design credit to original repository (linked above).
