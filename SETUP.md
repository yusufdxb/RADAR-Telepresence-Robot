# RADAR Setup Guide

## Prerequisites
- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.10+
- TurtleBot3 hardware or Gazebo simulation

## Installation

### 1. Install ROS 2 Jazzy
Follow the official guide: https://docs.ros.org/en/jazzy/Installation.html

### 2. Clone this repository
```bash
git clone https://github.com/yusufdxb/RADAR-Telepresence-Robot.git
cd RADAR-Telepresence-Robot
```

### 3. Install Python dependencies
```bash
pip install -r requirements.txt
```

### 4. Build the ROS 2 workspace
```bash
colcon build
source install/setup.bash
```

### 5. Launch RADAR
```bash
ros2 launch radar bringup.launch.py
```
