# ROS distribution to use
ARG ROS_DISTRO=humble

###########################################
# Overlay Image for TurtleBot3 Simulation #
###########################################
# FROM base AS overlay
FROM ros2_base AS overlay

ARG USERNAME=ros
ARG REPO_DIR=robot_worlds
USER root
# Create an overlay Colcon workspace

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && apt update

RUN apt install -y ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-dev-tools \
    python3-wstool \
    xterm

# install pip3
RUN wget https://bootstrap.pypa.io/get-pip.py -P ./src/${REPO_DIR}/docker/scripts && \
    cd ./src/${REPO_DIR}/docker/scripts && \
    python3 get-pip.py && PATH=$PATH:/home/ros/.local/bin

# RUN pip3 install transforms3d opencv-contrib-python==4.6.0.66 tinyspline

# Use Cyclone DDS as middleware
# RUN apt-get update && apt-get install -y --no-install-recommends \
#  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# robot specific installations

## turtlebot3
# $ sudo apt install ros-humble-dynamixel-sdk
# $ sudo apt install ros-humble-turtlebot3-msgs
# $ sudo apt install ros-humble-turtlebot3
RUN apt install -y ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-dynamixel-sdk

RUN mkdir -p /home/$USERNAME/overlay_ws/src
RUN chown $USERNAME /home/$USERNAME/overlay_ws

WORKDIR /home/$USERNAME/overlay_ws

# RUN source /overlay_ws/install/setup.bash \
#  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
#  && colcon build --symlink-install 

# Set up the entrypoint
COPY ./src/${REPO_DIR}/docker/entrypoint.sh /

USER $USERNAME

ENTRYPOINT [ "/entrypoint.sh" ]

# # ROS distribution to use
# ARG ROS_DISTRO=humble

# ###########################################
# # Overlay Image for TurtleBot3 Simulation #
# ###########################################
# # FROM base AS overlay
# FROM harmonic-humble AS overlay

# USER root
# # Create an overlay Colcon workspace
# RUN mkdir -p /overlay_ws/src
# RUN chown ros /overlay_ws
# USER ros
# WORKDIR /overlay_ws
# # COPY ./robots/ ./src/robots/
# # COPY ./worlds/ ./src/worlds/
# # COPY ./launch/ ./src/launch/



# # RUN source /turtlebot3_ws/install/setup.bash \
# #  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
# #  && colcon build --symlink-install

# # Set up the entrypoint
# COPY ./docker/entrypoint.sh /
# ENTRYPOINT [ "/entrypoint.sh" ]