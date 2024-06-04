#!/bin/bash

# WS_DIR=~/code/repos/bb_ws
REPO=robot_worlds
WS_DIR=$PWD
echo "moving into worskpace directory: $WS_DIR, before starting build ..."
cd $WS_DIR

# these arguments are based on names of the images defined in the dockerfiles.
# if changing here, update also in both:  ros2_base.Dockerfile & dev_overlay.Dockerfile 
ROS2_BASE_IMAGE=ros2_base
ROS2_DEV_IMAGE=dev_overlay

echo "[docker build] >> building ROS2 base image docker"
docker build -t $ROS2_BASE_IMAGE -f src/$REPO/docker/base_ros2_gazebo.Dockerfile .

echo "[docker build] >> building ros2 development overlay docker image"
docker build -t $ROS2_DEV_IMAGE -f src/$REPO/docker/worlds_overlay.Dockerfile .

echo "[docker build] >> Docker build of images: $ROS2_DEV_IMAGE and $ROS2_BASE_IMAGE is complete."
echo "[docker build] >> run: $ src/$REPO/docker/run_docker.sh"