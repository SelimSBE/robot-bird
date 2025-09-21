# robot-bird
Robot Bird is a simulation of a robot bird flight control


# Quick Start

## Requirements
- ROS 2 Humble
- Gazebo (Fortress or Garden)
- `gazebo_ros_pkgs` installed

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
````

## Build

Clone this repository into your ROS 2 workspace and build:

```bash
colcon build
source install/setup.bash
```

## Run Simulation

Launch Gazebo with the robotic bird:

```bash
ros2 launch robotic_bird description.launch.py
```


## Check Odometry

Verify odometry output:

```bash
ros2 topic echo /odom
```
