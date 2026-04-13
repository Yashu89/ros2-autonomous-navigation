# ROS2 Autonomous Robot Navigation and Object-Aware Exploration

This project implements an autonomous mobile robot system in ROS2 capable of mapping, navigation, and object-aware decision making in a simulated environment using Gazebo.

---

## Overview

The system integrates perception, localization, mapping, and navigation to enable a robot to explore an environment, detect objects, estimate their real-world positions, and adapt its movement accordingly.

---

## System Pipeline

Camera → Object Detection → Angle Estimation → LiDAR Fusion → Object Localization → Target Management → Navigation Control

---

## Features

- Autonomous exploration using LiDAR-based obstacle avoidance  
- Object detection using OpenCV (color-based detection)  
- Angle estimation from image coordinates  
- Object localization using LiDAR + odometry fusion  
- Target clustering and filtering for stable detection  
- Object-aware navigation behavior  
- Full system integration using ROS2 launch system  
- Custom TF handling for consistent odometry  

---

## Modules

### Perception
Detects objects in the camera feed and publishes their image-space position.

### Angle Estimation
Converts detected object pixel position into an angular offset relative to the robot.

### Object Localization
Combines LiDAR data, angle estimation, and odometry to compute real-world object coordinates.

### Target Manager
Maintains a stable list of detected objects by clustering repeated detections and filtering noise.

### Navigation
Implements autonomous exploration using:
- Obstacle avoidance  
- Wall-following behavior  
- Random exploration strategy  
- Object-aware motion adjustments  

### TF Handling
A custom odometry transformation is used to correct frame inconsistencies and maintain stable mapping.

---

## How to Run

### 1. Start Simulation Environment

Launch the Gazebo world:

```bash
gz sim -r slam_world.sdf
```

---

### 2. Spawn Robot

Run the robot bringup:

```bash
ros2 launch mobile_robot_bringup bot.launch.py
```

---

### 3. Start Full System

Launch perception, localization, and navigation:

```bash
ros2 launch mobile_robot_bringup full_system.launch.py
```

---

### 4. Visualize Map

Open RViz2 to view mapping and robot state:

```bash
rviz2
```

## Technologies Used

- ROS2  
- Gazebo  
- OpenCV  
- SLAM Toolbox  

---

## Project Structure

```
src/
 ├── mobile_robot_perception/
 ├── mobile_robot_navigation/
 ├── mobile_robot_bringup/
 ├── mobile_robot_description/
```

---

## Key Implementation Details

- Object detection is based on HSV color segmentation and contour filtering  
- Angle estimation converts pixel offset to angular deviation using camera field of view  
- Object localization fuses LiDAR range with angle and odometry  
- Navigation uses a state machine (forward, turn, follow, random exploration)  
- Target positions are refined using clustering and averaging over time  

---

## Future Work

- Implement frontier-based exploration for systematic environment coverage  
- Replace random exploration with goal-driven navigation  
- Extend perception module to support multiple object detection instead of single color-based detection  
- Integrate semantic perception for object classification and scene understanding  
- Improve handling of dynamic objects and real-time interaction strategies  

---

## Notes

This project focuses on system-level integration of multiple robotics components rather than isolated functionalities. It demonstrates how perception, mapping, and navigation can be combined into a cohesive autonomous system.
