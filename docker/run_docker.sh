#!/bin/bash
# Sample script to run a command in a Docker container
#
# Usage Example:
# ./run_docker.sh turtlebot3_behavior:overlay "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
# # Run the command
# docker run -it --net=host --ipc=host --privileged ${DOCKER_ARGS} "$1" bash -c "$2"

# DOCKER_IMAGE="harmonic-humble"
# docker user name
USER="ros"
DOCKER_IMAGE="dev_overlay"

# Define Docker volumes and environment variables
# mounting the local ros workspace directory to /home/ros/overlay_ws/ inside docker
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority" \
--volume="${PWD}/":"/home/ros/overlay_ws/":rw \
--volume="${HOME}/.bash_history:/home/${USER}/.bash_history"
"
DOCKER_ENV_VARS="
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
"
DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}

# Run the command
docker run -it --net=host --ipc=host --privileged ${DOCKER_ARGS} ${DOCKER_IMAGE} bash