# ROS distribution to use
ARG ROS_DISTRO=humble

## Base Image with ROS and Gazebo

# FROM nvidia/cudagl:9.0-base-ubuntu16.04
#FROM osrf/ros:kinetic-desktop-full-xenial
FROM osrf/ros:${ROS_DISTRO}-desktop as base

# Run a full upgrade and install utilities for development.
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    mesa-utils \
    vim \
    build-essential gdb \
    cmake cmake-curses-gui \
    git \
    ssh \
    wget \
 && rm -rf /var/lib/apt/lists/*

# Register the ROS package sources.
ENV UBUNTU_RELEASE=jammy
ENV GZ_DISTRO=gzharmonic
ENV ROS_DISTRO=${ROS_DISTRO}

RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $UBUNTU_RELEASE main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update

# # Install ROS2 Humble and its default gazebo pairing
# RUN apt-get install -y ros-${ROS_DISTRO}-ros-gz

# Install ROS2 Humble and gazebo harmonic (GZ_DISTRO=harmonic)
RUN apt-get install -y ros-${ROS_DISTRO}-ros-${GZ_DISTRO}

## Alternative Installation (ROS and gazebo separately)
# # Install ROS.
# RUN apt-get update && apt-get install -y \ 
#     ros-humble-desktop-full \
#  && rm -rf /var/lib/apt/lists/*

# # Upgrade Gazebo harmonic.
# RUN apt-get install lsb-release gnupg
# RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'
# RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
# RUN apt-get update && apt-get install -y \
#     gz-harmonic \
#  && rm -rf /var/lib/apt/lists/*

# # Initialize rosdep
# RUN rosdep init

# # Only for nvidia-docker 1.0
# LABEL com.nvidia.volumes.needed="nvidia_driver"
# # ENV PATH /usr/local/nvidia/bin:${PATH}
# # ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# # nvidia-container-runtime (nvidia-docker2)
# ENV NVIDIA_VISIBLE_DEVICES \
#    ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Some QT-Apps/Gazebo don't show controls without this
ENV QT_X11_NO_MITSHM 1

# Create users and groups.
ARG ROS_USER_ID=1000
ARG ROS_GROUP_ID=1000

RUN addgroup --gid $ROS_GROUP_ID ros \
 && useradd --gid $ROS_GROUP_ID --uid $ROS_USER_ID -ms /bin/bash -p "$(openssl passwd -1 ros)" -G root,sudo ros \
 && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
 && mkdir -p /workspace \
 && ln -s /workspace /home/workspace \
 && chown -R ros:ros /home/ros /workspace

USER ros
RUN rosdep update
WORKDIR /workspace

COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]