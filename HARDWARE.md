# RADAR Hardware Notes

This document records the hardware configuration and what each subsystem contributed to the prototype.

## Prototype Hardware

| Component | Purpose |
|---|---|
| Raspberry Pi | onboard compute |
| TurtleBot-class mobile base | ground locomotion |
| USB camera | live operator video |
| pan-tilt servo assembly | camera orientation |
| MAX30102 | pulse and SpO2 sensing |
| joystick | operator control input |
| custom 3D-printed mount | camera and sensor mounting |

## Public Assets

The repository already includes hardware-related media in `assets/`:
- `RADAR.png`
- `3D_Camera Mount.jpg`
- `Joystick.png`
- `Pulse Ox.png`

## Hardware-Validated Paths

These are the paths that were validated on the physical prototype before hardware access ended:
- joystick input to robot motion commands
- USB camera stream to ROS 2 topic
- pan-tilt servo control through ROS 2 nodes
- MAX30102 acquisition and vitals publishing

## Important Limitation

The repo should not imply that every software path was revalidated end to end on the final hardware. The Qt GUI exists in the public code, but the strongest hardware evidence remains the subsystem-level ROS 2 paths listed above.
