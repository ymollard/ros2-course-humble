FROM ros:jazzy-ros-base

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

# Gazebo Sim dependencies, Python support
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget -O- https://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && apt-get update && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
    libgz-transport14-dev \
    python3-gz-transport14

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

# Build Workspace
# RUN /bin/bash -c "source /opt/ros/"${ROS_DISTRO}"/setup.bash; colcon build --symlink-install"

# Add source-ing to .bashrc
RUN echo 'source /opt/ros/'${ROS_DISTRO}'/setup.bash' >> /home/${USERNAME}/.bashrc 
# RUN echo 'source /home/'${USERNAME}'/ros2_ws/install/setup.bash' >> /home/${USERNAME}/.bashrc

CMD ["bash"]
