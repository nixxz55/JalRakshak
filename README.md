# 🚰 JalRakshak – Automated Pipeline Inspection Rover

JalRakshak is a ROS 2 and Gazebo-based automated pipeline inspection rover simulation designed to detect water leakages inside underground pipeline environments.

The system uses a rover-mounted camera, OpenCV-based vision processing, odometry, and autonomous movement to identify simulated water leaks and estimate their locations.

---

## 🎯 Project Objective

The main objective of JalRakshak is to develop a robotic system that can:

- Inspect underground water pipelines
- Detect water leakage automatically
- Identify the approximate leak location
- Stop the rover for leak inspection
- Continue the inspection after detecting a leak
- Reduce water wastage and manual inspection effort

---

## ⚙️ Technologies Used

- ROS 2 Humble
- Gazebo
- Python
- OpenCV
- NumPy
- CvBridge
- URDF
- Differential Drive
- Odometry

---

## 🤖 Rover Features

- 4-wheel rover configuration
- Differential drive movement
- Onboard camera
- Autonomous forward movement
- Odometry-based distance tracking
- Vision-based water leak detection
- Multiple leak detection
- Automatic inspection stop
- Leak location estimation
- Position error calculation
- Mission completion detection

---

## 💧 Leak Detection

The rover uses a camera to detect blue-colored regions representing simulated water leakage.

OpenCV performs:

1. Image acquisition from ROS 2 camera topic
2. BGR to HSV conversion
3. Blue color filtering
4. Noise removal
5. Contour detection
6. Largest blue region detection
7. Leak confirmation

A leak is detected when the identified blue region exceeds the defined area threshold.

---

## 📍 Simulated Leak Locations

The current Gazebo environment contains five simulated leak points:

| Leak | Location |
|------|----------|
| Leak 1 | 10 m |
| Leak 2 | 20 m |
| Leak 3 | 30 m |
| Leak 4 | 40 m |
| Leak 5 | 50 m |

The rover compares its odometry-based detected position with the expected leak position and calculates the position error.

---

## 🔄 System Workflow

```text
Start
  ↓
Gazebo Pipeline Environment
  ↓
Spawn JalRakshak Rover
  ↓
5 Second Startup Delay
  ↓
Autonomous Movement
  ↓
Camera Image Processing
  ↓
Blue Region Detection
  ↓
Water Leak Detected?
  ├── No → Continue Moving
  │
  └── Yes
       ↓
   Stop Rover
       ↓
   Calculate Leak Location
       ↓
   Calculate Position Error
       ↓
   5 Second Inspection
       ↓
   Continue Mission
       ↓
   Next Leak
       ↓
   All Leaks Inspected
       ↓
   Mission Complete
