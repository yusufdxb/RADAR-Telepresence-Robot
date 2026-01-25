# RADAR — Remote Autonomous Doctor Assistance Robot

RADAR (Remote Autonomous Doctor Assistance Robot) is a **ROS 2–based medical telepresence robotic system** designed to enable remote clinician interaction, real-time robot control, live video streaming, and patient vital sign monitoring in environments where physical presence is limited.

The system was developed as a **complete, functional robotic platform** emphasizing modularity, low-cost hardware, and reliable remote operation.

---

## 🚑 Motivation & Problem Statement

Access to medical personnel can be limited in remote, hazardous, or high-risk environments. RADAR addresses this challenge by providing a mobile telepresence robot that allows clinicians to:

- Remotely navigate a physical environment  
- Observe patients through live video  
- Monitor basic physiological data in real time  

The platform is designed for **research, prototyping, and educational use**, with a strong emphasis on extensibility.

---

## ✨ System Capabilities

- Real-time joystick-based teleoperation  
- Live camera streaming with pan–tilt control  
- Onboard vital sign monitoring (pulse & SpO₂)  
- Modular ROS 2 node-based architecture  
- Designed for rapid prototyping and extension  

---

## 🧱 System Architecture

RADAR is implemented as a set of **loosely coupled ROS 2 nodes**, enabling clean separation of perception, control, and sensing:

- **Teleoperation Node**  
  Reads joystick input and publishes velocity commands  

- **Base Controller Node**  
  Drives the mobile robot via `/cmd_vel`  

- **Camera Stream Node**  
  Publishes live video frames from the onboard camera  

- **Pan–Tilt Node**  
  Controls servo-driven camera orientation  

- **Vitals Node (MAX30102)**  
  Reads and publishes pulse and SpO₂ data  

This architecture allows individual subsystems to be modified or replaced without affecting the overall system.

---

## 🔌 Hardware Components

- Mobile robot base (TurtleBot-class platform)  
- Raspberry Pi  
- USB camera  
- Pan–tilt servo module  
- MAX30102 pulse oximeter  
- Joystick controller  

---

## 💻 Software Stack

- Ubuntu Linux  
- ROS 2 (Humble)  
- Python and C++  
- OpenCV  
- Qt (GUI development)  

---

## 📂 Repository Structure

