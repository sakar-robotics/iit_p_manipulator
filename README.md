# IGUS Rebel Cobot 6DOF Arm with ROS2 and MoveIt2

## Introduction

This project contains ROS2 Humble code integrated with MoveIt2 (main branch) for the IGUS Rebel Cobot 6DOF Arm.

## Table of Contents

- [IGUS Rebel Cobot 6DOF Arm with ROS2 and MoveIt2](#igus-rebel-cobot-6dof-arm-with-ros2-and-moveit2)
  - [Introduction](#introduction)
  - [Table of Contents](#table-of-contents)
  - [Repo Structure](#repo-structure)
  - [Installation Requirements](#installation-requirements)
    - [ROS2 Humble Installation](#ros2-humble-installation)
    - [MoveIt2 Installation from Source (Main Branch)](#moveit2-installation-from-source-main-branch)
  - [Launch \& Control Details](#launch--control-details)
  - [Usage Instructions](#usage-instructions)
    - [Using with PC (Ethernet Connection)](#using-with-pc-ethernet-connection)
    - [Using with Jetson Platform](#using-with-jetson-platform)
  
## Repo Structure

- **igus_rebel**: Contains hardware interfaces to communicate with the robotic arm and teleoperation nodes (e.g., keyboard and joystick control).
- **igus_rebel_description**: Provides URDF files and meshes for the robotic arm.
- **igus_rebel_moveit_config**: Houses configuration and launch files for integrating the arm with MoveIt2.
- **igus_rebel_msgs**: Contains custom ROS interfaces and message definitions for the system.
  
## Installation Requirements

### ROS2 Humble Installation

For ROS2 Humble installation on Ubuntu, please refer to the official docs:  
[ROS2 Humble Ubuntu Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### MoveIt2 Installation from Source (Main Branch)

MoveIt 2 source installation requires various other tools. Run the following commands:
  
```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

sudo apt update
sudo apt dist-upgrade
rosdep update

source /opt/ros/$ROS_DISTRO/setup.bash

# Remove any pre-existing MoveIt2 debians to avoid conflicts:
sudo apt remove ros-$ROS_DISTRO-moveit*****

# Create a colcon workspace:
export COLCON_WS=~/ws_moveit2/
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src

# Download the MoveIt2 source repository:
git clone https://github.com/moveit/moveit2.git -b main

# Import additional repositories if available:
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done

rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd $COLCON_WS
MAKEFLAGS="-j4 -l1" colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential

# Source the colcon workspace:
source $COLCON_WS/install/setup.bash
```

## Launch & Control Details

This repository provides several launch files and scripts to bring up the robotic arm and control it using MoveIt2 and ROS 2 control. Below is a detailed explanation of the main components:

- **igus_arm_bringup.launch.py**  
  This is the main bringup launch file that starts the entire system. It includes:
  - **Hardware Interface and ROS2 Control**:  
    The file `rebel.launch.py` initializes low-level control by loading the robot’s URDF via Xacro, setting up the `ros2_control_node` (which includes the hardware interface) and the `robot_state_publisher`. It then spawns necessary controllers such as the joint state broadcaster and the arm’s trajectory controller.
  - **MoveIt2 Configuration**:  
    The file `move_group.launch.py` handles MoveIt2 functionalities. It configures:
    - Motion planning settings (joint limits, OMPL planners, kinematics, etc.)
    - Servo configuration parameters to support servoing.
    - Controller settings via MoveIt’s simple controller manager.
  - **Control Modes for Servoing**:  
    Two nodes are provided for manual control:
    - **Keyboard Input (keyboard_input)**:  
      Sends servo commands via keyboard to change poses or joint velocities.
    - **Joystick Input (joystick_input)**:  
      Allows intuitive, continuous control using a joystick.
  
- **Usage Overview**:  
  To run the system:
  1. Launch the main bringup file (`igus_arm_bringup.launch.py`). Adjust launch arguments to choose components:
     - `use_gui`: Whether to start RViz.
     - `launch_mode`: Options are `move_group`, `servo`, or `both`.
     - `servo_control`: Set to `keyboard` or `joystick` for sending servo commands.
  2. Hardware interface aspects are managed by `rebel.launch.py`, while MoveIt2 settings are configured by `move_group.launch.py`.
  3. Use the appropriate control script (keyboard_input or joystick_input) to command the arm via MoveIt Servo.
  
## Usage Instructions

### Using with PC (Ethernet Connection)

1. **Network Configuration**:  
   Configure your network to connect to the robot via Ethernet with the IP address **192.168.3.10**. Verify connectivity by pinging the IP:

   ```bash
   ping 192.168.3.10
   ```

2. **Launch the System**:  
   Run the bringup launch command on your PC:

   ```bash
   ros2 launch igus_rebel igus_arm_bringup.launch.py use_gui:=true launch_mode:=both servo_control:=keyboard
   ```

   (Modify the `servo_control` argument as needed.)  
3. **In RViz**:  
   Once the system is up and RViz is launched, set a desired pose, then click "Plan" and "Execute" to move the arm.

### Using with Jetson Platform

1. **On the Robot (Jetson)**:  
   SSH into the robot and run the bringup launch file without RViz:

   ```bash
   ros2 launch igus_rebel igus_arm_bringup.launch.py use_gui:=false launch_mode:=both servo_control:=joystick
   ```

2. **On Your Development PC**:  
   Run the RViz-only launch to visualize and interact with the system:

   ```bash
   ros2 launch igus_rebel_moveit_config moveit_rviz.launch.py
   ```
