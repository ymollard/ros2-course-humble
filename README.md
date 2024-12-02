# ros2-course for CREMI
DevContainer setup for ROS2 course

## Prerequisites

This is a Dev container of **ROS 2 Humble + PAL Tiago robot** for Visual Studio Code for CREMI @Bordeaux University.

## Start DevContainer
```bash
git clone https://github.com/ymollard/ros2-course-humble -b cremi
./ros2-course-humble/start_ros_humble.bash
```
This script sets the right configuration to user containers at CREMI and starts the container in Visual Studio Code.

When VSCode opens, trust the sources and **accept the installation of the Dev Container extension**.

## Installing in Container
The `~/ros2_ws/src` directory in the container is bound to the `src` directory in this repository. Which means source installs are persistent and saved when stopping and/or rebuilding the DevContainer.
The opposite is true for binary installs, these are not persistent. Unless added to the Dockerfile. 

ROS2 packages should be installed/cloned into the `~/ros2_ws/src` folder. 
To install the dependencies for these packages use:
``` bash
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y -r
```

## Start Tiago 2D navigation
To build the workspace use:
``` bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Start Gazebo simulation of Tiago robot:
``` bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```

In a new terminal, start cartographer to run SLAM:
``` bash
ros2 launch tiago_2dnav tiago_nav_bringup.launch.py slam:=True is_public_sim:=True
```

In a new terminal, start teleoperation from the keyboard:
``` bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Close teleoperation with Ctrl+C and save map to file `our_map` using:
``` bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/pal_maps/maps/our_map/map
```
Start 2D navigation to any goal pose using:
``` bash
ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=our_map
```
- Estimate the 2D pose
- Send a 2D nav goal

