# RADAR GUI

This directory contains the Qt 6 operator interface for the RADAR telepresence robot.

## What It Is

The GUI is intended to give a remote operator one place to:
- view the camera stream
- monitor vitals
- control camera pan and tilt
- observe system state without relying on a terminal

## Validation Status

The GUI source is public in this repository, but it was not fully revalidated on the final hardware after access ended. The stronger verified path in this project remains the ROS 2 subsystem nodes documented in the root README.

## Why It Still Matters

Even without final demo media, the GUI is useful portfolio signal because it shows the project was designed as an operator-facing system rather than a collection of backend nodes.
