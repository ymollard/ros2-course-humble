FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install some dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    wget \
    x11-xserver-utils \
    software-properties-common \
    python3-pip \
    xauth \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-srdfdom \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-kinematics-interface \
    ros-${ROS_DISTRO}-ur-msgs \
    ros-${ROS_DISTRO}-ur-client-library \
    ros-${ROS_DISTRO}-ur-robot-driver \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-can-msgs \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-joy-linux \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-gripper-controllers \
    ros-${ROS_DISTRO}-omni-base-description \
    ros-${ROS_DISTRO}-pal-robotiq-controller-configuration  \
    ros-${ROS_DISTRO}-pal-robotiq-description  \
    ros-${ROS_DISTRO}-pal-navigation-cfg-bringup  \
    ros-${ROS_DISTRO}-pal-navigation-cfg-params  \
    ros-${ROS_DISTRO}-pmb2-laser-sensors  \
    ros-${ROS_DISTRO}-nav2-bringup  \
    ros-${ROS_DISTRO}-urdf-test  \
    ros-${ROS_DISTRO}-pmb2-2dnav  \
    ros-${ROS_DISTRO}-tiago-description  \
    ros-${ROS_DISTRO}-tiago-controller-configuration  \
    ros-${ROS_DISTRO}-tiago-bringup  \
    ros-${ROS_DISTRO}-moveit-ros-control-interface  \
    ros-${ROS_DISTRO}-moveit-ros-perception  \
    ros-${ROS_DISTRO}-pmb2-description  \
    ros-${ROS_DISTRO}-pal-hey5-description  \
    ros-${ROS_DISTRO}-pal-gripper-description  \
    ros-${ROS_DISTRO}-tiago-2dnav  \
    ros-${ROS_DISTRO}-tiago-laser-sensors  \
    ros-${ROS_DISTRO}-moveit-servo  \
    ros-${ROS_DISTRO}-play-motion2-msgs  \
    ros-${ROS_DISTRO}-gazebo-plugins  \
    ros-${ROS_DISTRO}-gazebo-ros2-control  \
    ros-${ROS_DISTRO}-pal-gazebo-worlds  \
    ros-${ROS_DISTRO}-pal-gazebo-plugins  \
    ros-${ROS_DISTRO}-tiago-moveit-config liburdfdom-tools  \
    ros-${ROS_DISTRO}-tiago-gazebo  \
    ros-${ROS_DISTRO}-pal-hey5-controller-configuration  \
    ros-${ROS_DISTRO}-joy-teleop  \
    ros-${ROS_DISTRO}-pmb2-controller-configuration  \
    ros-${ROS_DISTRO}-pmb2-bringup  \
    ros-${ROS_DISTRO}-pal-gripper-controller-configuration  \
    ros-${ROS_DISTRO}-omni-base-controller-configuration  \
    ros-${ROS_DISTRO}-play-motion2  \
    ros-${ROS_DISTRO}-teleop-tools-msgs  \
    ros-${ROS_DISTRO}-moveit-planners-chomp \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    '~nros-${ROS_DISTRO}-rqt*' \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

ENV SHELL=/bin/bash

USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/${USERNAME}/ros2_ws/src
WORKDIR /home/${USERNAME}/ros2_ws
# Install rosdeps from packages
RUN sudo apt update && rosdep update \
    && rosdep install --from-paths src --ignore-src -y -r

# Add colcon mixin default
RUN /bin/bash -c "source /opt/ros/"${ROS_DISTRO}"/setup.bash; colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml; colcon mixin update default"

# Update deps with rosdep
WORKDIR /home/${USERNAME}/ros2_ws
# Install rosdeps from packages
RUN sudo apt update && rosdep update \
    && rosdep install --from-paths src --ignore-src -y -r

# Build Workspace
WORKDIR /home/${USERNAME}/ros2_ws/
RUN /bin/bash -c "source /opt/ros/"${ROS_DISTRO}"/setup.bash; colcon build --symlink-install"

# Add source-ing to .bashrc
RUN echo 'source /opt/ros/'${ROS_DISTRO}'/setup.bash' >> /home/${USERNAME}/.bashrc 
# RUN echo 'source /home/'${USERNAME}'/ros2_ws/install/setup.bash' >> /home/${USERNAME}/.bashrc

CMD ["bash"]
