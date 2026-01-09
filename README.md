# RADAR – Remote Autonomous Doctor Assistance Robot

RADAR (Remote Autonomous Doctor Assistance Robot) is a ROS 2–based telepresence robotic system designed to enable remote medical interaction and monitoring in environments where physical presence is limited.

The system allows a remote clinician to manually control a mobile robot, view live video, and monitor patient vital signs in real time.

---

## Project Overview

RADAR is built on a mobile robotic platform and integrates:
- Real-time video streaming
- Remote joystick-based teleoperation
- Pan-tilt camera control
- Vital sign monitoring using onboard sensors
- A modular ROS 2 software architecture

The project focuses on low-cost hardware, rapid prototyping, and reliable remote operation.

---

## System Features

- ROS 2–based architecture
- Live camera feed with pan-tilt control
- Remote joystick teleoperation
- Onboard pulse oximeter for vital signs
- Modular software nodes for easy expansion
- Designed for research and educational use

---

## Hardware Components

- Mobile robot base (TurtleBot-class platform)
- Raspberry Pi
- USB camera
- Pan-tilt servo module
- MAX30102 pulse oximeter
- Joystick controller

---

## Software Stack

- Ubuntu Linux
- ROS 2
- Python and C++
- OpenCV
- Qt (for GUI development)

---

## Repository Structure

RADAR-Telepresence-Robot/

├── src/ # ROS 2 packages

├── launch/ # Launch files

├── config/ # Configuration files

├── assets/ # Images and videos

└── README.md

---
## Project Status

This project is currently under active development as part of a graduate-level robotics course.
Core teleoperation and video streaming features are functional.
Additional work is ongoing to improve robustness, user interface, and sensor integration.

---

---

## Installation

This project is developed using ROS 2 on Ubuntu Linux.

### Prerequisites
- Ubuntu 22.04 (recommended)
- ROS 2 (Humble)
- Python 3
- colcon

### Setup
```bash
# Create a workspace
mkdir -p ~/radar_ws/src
cd ~/radar_ws/src

# Clone the repository
git clone https://github.com/yusufdxb/RADAR-Telepresence-Robot.git

# Build the workspace
cd ..
colcon build
source install/setup.bash


Usage

The RADAR system is composed of multiple ROS 2 nodes for:

Mobile base teleoperation

Camera streaming

Pan-tilt camera control

Vital sign sensing

Launch files will be provided in the launch/ directory once integration is complete.


5. Click **Commit changes**

That’s it.

---

## Why this step matters
Even without code, this tells:
- Professors: *you understand workflow*
- Recruiters: *you know ROS structure*
- Teammates: *how to start*

This is **portfolio-level documentation**.

---

## What comes AFTER this (you choose)

Reply with **one** of these and we continue:

1. **“Add system architecture diagram”** (block diagram, nodes, topics)
2. **“Add software architecture section”** (nodes, topics, data flow)
3. **“Upload ROS code”**
4. **“Polish README for portfolio”**

You’re doing fine. We’re building this correctly, not fast and sloppy.



