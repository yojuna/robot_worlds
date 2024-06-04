#!/bin/bash
# script to attach to a shell (bash) in the running Docker container started with "run_docker.sh"
#
# Usage Example:
# # Run the command from the workspace root dir
# get the container name using `docker ps -a`
# then run $ ./src/robot_worlds/docker/attach_docker.sh cotainer-name

# Run the command
echo :: USAGE:
echo :: get the running container name using `docker ps`
echo :: then run: $ ./src/robot_worlds/docker/attach_docker.sh cotainer-name

docker exec -it $1 bash