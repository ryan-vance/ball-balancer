# Hardware Documentation

## Original Design

This project uses the mechanical platform from the **[Ball-Balancer-V2](https://github.com/aaedmusa/Ball-Balancer-V2)** repository by aaedmusa.

All hardware design credit goes to the original creators. We used their CAD files and bill of materials to construct the physical platform.

---

## Quick Links

### CAD Files & Bill of Materials
- [View all CAD files in original repository](https://github.com/aaedmusa/Ball-Balancer-V2)
- Files include STL models for 3D printing the delta robot frame

---

## Key Components Used

Based on the original BOM, our build included:

**Electronics:**
- Teensy 4.1 (microcontroller)
- 3x NEMA 17 Stepper Motors
- 3x TMC2208 Stepper Motor Drivers  
- Resistive Touchscreen (4-wire)
- 12V Power Supply
- Solderable breadboard for connections

**Mechanical:**
- 3D Printed Frame Components (from original CAD files)
- Ball bearing
- Connecting rods
- Platform assembly
- Hardware (screws, nuts, bolts)

**Sensor:**
- Resistive touchscreen for ball position detection

---

## Our Hardware Modifications

- Our team modified certain CAD files based on printer variations
- Created unique touchscreen brackets to secure the touchpad without triggering the touchpad

---

## Assembly Notes

For complete assembly instructions, refer to the [Ball Balancer Instructables Page](https://www.instructables.com/Ball-Balancer/).

**Additional notes from our build:**

[PLACEHOLDER - Add any lessons learned, tips, or issues you encountered]

Examples:
- Teensy microcontroller 5V power bridge must be severed to power the ball balancer and program the Teensy concurrently
- Pressure touchscreen requires large force to trigger
- Pressure touchscreen tends to receive false readings often, be careful of bracket placement
- Stepper motor & power converter tuning is critical - verify before final assembly
- Cable and heat management - each motor have long 1m cable cut, as the full length creates a risk of wire damage because the cables are looped near driver heatsinks
- Current deprivation: wiring the motors in parallel led to current deprivation, current was set to 3A to properly supply the circuit 

---

## Credits

Hardware platform design: [Ball-Balancer-V2](https://github.com/aaedmusa/Ball-Balancer-V2) by aaedmusa

Our team implemented the control system software on this platform.
