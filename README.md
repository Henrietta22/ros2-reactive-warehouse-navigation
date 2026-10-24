# Scaled Mobile Robot Reactive Navigation in Gazebo

Autonomous reactive navigation implementation using ROS 2 Humble and Gazebo Classic. The system is evaluated in the AWS RoboMaker Small Warehouse environment with a scaled differential-drive robot model.

## Overview

The package processes planar laser scan returns to compute velocity commands for collision-free passage through warehouse corridors. The default TurtleBot3 Waffle platform was modified to a 1.5x scale to evaluate navigation performance with wider chassis geometry in constrained passages.

## Environment & Dependencies

- Ubuntu 22.04 LTS / WSL2
- ROS 2 Humble Hawksbill
- Gazebo Classic 11
- aws-robomaker-small-warehouse-world

## Setup & Execution

### 1. Model and Shader Configuration

```bash
source /usr/share/gazebo-11/setup.bash 2>/dev/null || source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/usr/share/gazebo-11/media:/usr/share/gazebo-11/media/materials:$HOME/ros2_ws/src/aws-robomaker-small-warehouse-world:$GAZEBO_RESOURCE_PATH
export GAZEBO_MODEL_PATH=$HOME/ros2_ws/src/aws-robomaker-small-warehouse-world/models:~/.gazebo/models:$GAZEBO_MODEL_PATH
```

### 2. Launch Simulation

```bash
gazebo --verbose $HOME/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
```

### 3. Spawn Scaled Model

```bash
ros2 run gazebo_ros spawn_entity.py   -entity custom_waffle   -file ~/.gazebo/models/custom_waffle/model.sdf   -x 0.0 -y -1.5 -z 0.1
```

### 4. Run Controller

```bash
cd ~/ros2_ws/src/reactive_navigation
python3 obstacle_avoidance.py
```

## Control Logic

The node subscribes to `/scan` and publishes `geometry_msgs/msg/Twist` on `/cmd_vel`.

1. **Sector Segmentation:** LiDAR ranges are segmented into front, left, and right evaluation windows.
2. **Clearance Check:** When obstacle distance in the forward sector drops below the clearance threshold (0.65 m), the robot switches from linear progression to rotational alignment.
3. **Recovery:** Rotational direction is selected based on the sector with greater clearance until the forward corridor is clear.
