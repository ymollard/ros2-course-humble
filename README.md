# ros2-course
DevContainer setup for ROS2 course


## HOW TO USE

### Prerequisites
- Docker Engine
- VS Code
- VS Code extension: Dev Containers

### Start DevContainer
1. Open this repository in VS Code.
2. `crtl + shift + P` to open command prompt
3. Enter `Dev Containers: Rebuild and Reopen in Container` 

VS Code will start the Dev Container building the Dockerfile found in the `/docker` dir. This is a basic ROS2 Jazzy Container with the following ROS related binaries installed:
- `ros-jazzy-plotjuggler-ros`, a topic plotter package.
- `ros-jazzy-moveit`, binary install of the base MoveIt2 packages
- `ros-jazzy-ros-gz`, binary install of gz-sim11 Gazebo Harmonic ROS2
- `'~nros-jazzy-rqt*'`, RQT for ROS2 Jazzy

### Installing into Container
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

### Start BFFP Demo Factory
After building and sourcing the workspace, use: `ros2 launch bffp_gazebo factory_moveit_full.launch.py` to launch the demo. This launches a Gazebo simulation and a HC10DTP with full MoveIt2 support. Rviz2 is also launched to visualize ROS2 topics.