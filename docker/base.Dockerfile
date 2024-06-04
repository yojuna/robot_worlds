# ROS distribution to use
ARG ROS_DISTRO=humble

########################################
# Base Image for TurtleBot3 Simulation #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
# ENV WS_DIR=${WS_DIR}
ENV UBUNTU_RELEASE=jammy
ENV GZ_DISTRO=gzharmonic
ARG WS_DIR=ros_ws
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
    python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine \
    mesa-utils \
    vim \
    build-essential gdb \
    cmake cmake-curses-gui \
    git \
    ssh \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Register the ROS package sources.
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $UBUNTU_RELEASE main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# # Install ROS2 Humble and its default gazebo pairing
# RUN apt-get install -y ros-${ROS_DISTRO}-ros-gz

# Install ROS2 Humble and gazebo harmonic (GZ_DISTRO=harmonic)
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros-${GZ_DISTRO}

# Install additional Python modules
RUN pip3 install matplotlib transforms3d

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create Colcon workspace with external dependencies
RUN mkdir -p /${WS_DIR}/src
WORKDIR /${WS_DIR}/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /${WS_DIR}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

ENV TURTLEBOT3_MODEL=waffle_pi

# # Download Groot2 AppImage and place it in the home folder.
# WORKDIR /root/
# RUN curl -o Groot2.AppImage https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.0.1-x86_64.AppImage \
#  && chmod a+x Groot2.AppImage

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
WORKDIR /${WS_DIR}
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

###########################################
# Overlay Image for TurtleBot3 Simulation #
###########################################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
# COPY ./tb3_autonomy/ ./src/tb3_autonomy/
# COPY ./tb3_worlds/ ./src/tb3_worlds/
RUN ls -la ./
COPY ./robots/ ./src/robots/
COPY ./worlds/ ./src/worlds/
COPY ./launch/ ./src/launch/
COPY ./CMakeLists.txt ./
RUN source /${WS_DIR}/install/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay as dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
 gdb gdbserver nano

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /overlay_ws/

# # Move Groot2 to new user's home directory and ensure it can be run
# RUN groupadd fuse \
#  && usermod -aG fuse ${USERNAME}
# RUN mv /root/Groot2.AppImage /home/${USERNAME} \ 
#  && chown ${UID}:${GID} /home/${USERNAME}/Groot2.AppImage

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc