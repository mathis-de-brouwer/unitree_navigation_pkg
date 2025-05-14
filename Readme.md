# Unitree Navigation Package User Guide

This document explains how to use the unitree_navigation_pkg for computer vision-based navigation with a TurtleBot3 in various configurations.

## Table of Contents
1. [System Overview](#system-overview)
2. [Installation Prerequisites](#installation-prerequisites)
3. [Basic Usage](#basic-usage)
4. [Using Physical Camera](#using-physical-camera)
5. [Using Video File](#using-video-file)
6. [Advanced Configuration](#advanced-configuration)
7. [Troubleshooting](#troubleshooting)

## System Overview

The unitree_navigation_pkg provides:
- Path detection using YOLOv8 segmentation models
- Computer vision-based navigation for TurtleBot3 in Gazebo simulation
- Support for both physical cameras and pre-recorded videos
- Visualization of the segmentation and navigation decision making

## Installation Prerequisites

Ensure you have the following installed:
- ROS 2 Humble
- Gazebo 11
- TurtleBot3 packages
- OpenCV
- Ultralytics YOLOv8

```bash
# Install YOLOv8
pip install ultralytics

# Install other dependencies
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description 
sudo apt install ros-humble-cv-bridge python3-opencv

Basic Usage
1. Start TurtleBot3 in Gazebo (Required for all methods)
In a terminal, launch the TurtleBot3 in Gazebo:

export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo empty_world.launch.py

2. Check if TurtleBot3 responds to commands
Test direct control with:

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

Using Physical Camera
This method uses your physical camera for real-time path detection.

Launch with Camera Bridge
cd ~/ros2_ws
source install/setup.bash
ros2 launch unitree_navigation_pkg simple_camera_nav.launch.py

Options:
Different camera ID: ros2 launch unitree_navigation_pkg simple_camera_nav.launch.py camera_id:=1
Different model: ros2 launch unitree_navigation_pkg simple_camera_nav.launch.py model_path:=/path/to/model.pt
Change command topic: ros2 launch unitree_navigation_pkg simple_camera_nav.launch.py cmd_topic:=/custom/cmd_vel
Using Video File
This method uses a pre-recorded video for navigation testing.

Launch with Video Navigation

cd ~/ros2_ws
source install/setup.bash
ros2 launch unitree_navigation_pkg unitree_navigation.launch.py use_camera:=false
Options:
Custom video file: ros2 launch unitree_navigation_pkg unitree_navigation.launch.py use_camera:=false video_path:=/path/to/your/video.mp4
Different model: ros2 launch unitree_navigation_pkg unitree_navigation.launch.py use_camera:=false model_path:=/path/to/model.pt
Advanced Configuration
Launch Just Segmentation Node
If you only want to test the segmentation without navigation:

ros2 run unitree_navigation_pkg segmentation_node

Launch Just Navigation Node
If you want to run the navigation node separately:

ros2 run unitree_navigation_pkg navigation_node
Launch Video Tester Only
To test with a video without controlling a robot:
ros2 launch unitree_navigation_pkg gazebo_navigation.launch.py
Full Integration (Gazebo + Video)
To launch both Gazebo and video-based navigation:

cd ~/ros2_ws
colcon build --packages-select unitree_navigation_pkg
source install/setup.bash
# First terminal: Launch Gazebo
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Second terminal: Launch Video Navigation
ros2 launch unitree_navigation_pkg unitree_navigation.launch.py use_camera:=false

Component-Specific Commands
Camera Bridge:
# Run camera bridge only with default camera (0)
ros2 run unitree_navigation_pkg camera_bridge

# Run camera bridge with a specific camera
ros2 run unitree_navigation_pkg camera_bridge --ros-args -p camera_id:=1

# Run camera bridge with a specific model
ros2 run unitree_navigation_pkg camera_bridge --ros-args -p model_path:=/path/to/model.pt
Video Navigation Tester:
# Run with default video
ros2 run unitree_navigation_pkg video_navigation_tester

# Run with specific video
ros2 run unitree_navigation_pkg video_navigation_tester --ros-args -p video_path:=/path/to/video.mp4

# Run with specific playback speed (half speed)
ros2 run unitree_navigation_pkg video_navigation_tester --ros-args -p playback_speed:=0.5
Navigation Node:
# Run with default parameters
ros2 run unitree_navigation_pkg navigation_node

# Run with custom speed and turning angle
ros2 run unitree_navigation_pkg navigation_node --ros-args -p constant_speed:=0.3 -p turning_angle:=0.5
