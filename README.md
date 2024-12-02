# ros2-course
DevContainer setup for ROS2 course


## Prerequisites

### Linux
Install the following:
- Docker Engine
- VS Code
- VS Code extension: Dev Containers

Follow the listed steps to finish setup:
1. Create a json file in the `.devcontainer` dir named `devcontainer.json` and copy the content of `devcontainer - linux.json` into it. 

### Windows
Install the following:
- Docker Engine
- VS Code
- VS Code extension: Dev Containers
- XLaunch or other Xserver

Follow the listed steps to finish setup:
1. Create a json file in the `.devcontainer` dir named `devcontainer.json` and copy the content of `devcontainer - windows.json` into it. 
2. Start a Xserver with 0.0 as DISPLAY port

## Start DevContainer
1. Open this repository in VS Code.
2. `crtl + shift + P` to open command prompt
3. Enter `Dev Containers: Rebuild and Reopen in Container` 

VS Code will start the Dev Container building the Dockerfile found in the `/docker` dir. This is a basic ROS2 Humble Container + PAL Tiago binaries. 

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
This should install all package dependencies referenced in the `package.xml` files.
To build the workspace use:
``` bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Start BFFP Demo Factory
After installing the rosdeps and building and sourcing the workspace (see [Installing in Container](#installing-in-container)), use: `ros2 launch bffp_gazebo factory_moveit_full.launch.py` to launch the demo. This launches a Gazebo simulation and a HC10DTP with full MoveIt2 support. Rviz2 is also launched to visualize ROS2 topics.

``` bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Start Tiago 2D navigation
Start Gazebo simulation of Tiago:
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

