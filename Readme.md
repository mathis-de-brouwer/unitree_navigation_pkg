# Unitree Navigation Package

The `unitree_navigation_pkg` enables computer vision-based navigation for TurtleBot3 robots using YOLOv8 segmentation models. It supports both simulation in Gazebo and real-world applications with physical cameras or video files.

## Table of Contents

1. [System Overview](#system-overview)
2. [Installation Prerequisites](#installation-prerequisites)
3. [Basic Usage](#basic-usage)
4. [Using a Physical Camera](#using-a-physical-camera)
5. [Using a Video File](#using-a-video-file)
6. [Advanced Configuration](#advanced-configuration)
7. [Troubleshooting](#troubleshooting)

## System Overview

This package offers:

- Path detection using YOLOv8 segmentation models.
- Computer vision-based navigation for TurtleBot3 in Gazebo simulation.
- Support for both physical cameras and pre-recorded videos.
- Visualization of segmentation and navigation decision-making processes.

## Installation Prerequisites

Ensure the following are installed:

- ROS 2 Humble
- Gazebo 11
- TurtleBot3 packages
- YOLOv8 segmentation models

## Basic Usage

1. **Clone the Repository:**

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/mathis-de-brouwer/unitree_navigation_pkg.git
   ```

2. **Install Dependencies:**

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Package:**

   ```bash
   colcon build
   ```

4. **Source the Workspace:**

   ```bash
   source install/setup.bash
   ```

5. **Launch the Simulation:**

   ```bash
   ros2 launch unitree_navigation_pkg simulation_launch.py
   ```

## Using a Physical Camera

1. **Connect the Camera:**

   Ensure your physical camera is connected and recognized by the system.

2. **Launch the Camera Node:**

   ```bash
   ros2 launch unitree_navigation_pkg camera_launch.py
   ```

3. **Start Navigation:**

   ```bash
   ros2 launch unitree_navigation_pkg navigation_launch.py
   ```

## Using a Video File

1. **Specify the Video Path:**

   Edit the `video_launch.py` file to set the path to your video file.

2. **Launch the Video Node:**

   ```bash
   ros2 launch unitree_navigation_pkg video_launch.py
   ```

3. **Start Navigation:**

   ```bash
   ros2 launch unitree_navigation_pkg navigation_launch.py
   ```

## Advanced Configuration

- **Model Parameters:**

  Adjust the YOLOv8 model parameters in the `config` directory to fine-tune detection performance.

- **Navigation Settings:**

  Modify navigation parameters in the `navigation` directory to suit different environments or robot configurations.

## Troubleshooting

- **Camera Not Detected:**

  Ensure the camera is properly connected and recognized by the system. Use `ls /dev/video*` to check.

- **Simulation Issues:**

  Verify that Gazebo is installed correctly and that the TurtleBot3 model is properly configured.

- **Model Loading Errors:**

  Confirm that the YOLOv8 models are placed in the correct directory and that the paths in the configuration files are accurate.
