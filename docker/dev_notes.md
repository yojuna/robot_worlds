# overview

## simple setup

Build base and overlay docker with helper script
```
cd WS_DIR
./src/robot_worlds/docker/build_docker.sh
```
Run the overlay container, mounted with the current workspace directory
```
./src/robot_worlds/docker/run_docker.sh
```

Once the overlay workspace docker container is ready, it should look something like this: 
```
nemo@homelab:~/code/repos/ws_robot_worlds$ ./src/robot_worlds/docker/run_docker.sh 
Sourced ROS 2 humble
ros@homelab:~/overlay_ws$ ls -la src/
total 24
drwxrwxr-x  6 ros ros 4096 Jun  3 15:24 .
drwxrwxr-x  6 ros ros 4096 Apr  8 18:23 ..
drwxrwxr-x  9 ros ros 4096 Jun  3 15:29 robot_worlds
ros@homelab:~/overlay_ws$ 
```

## building the docker images
### base image

builds on the OSRF humble-desktop-full docker image 
ref: https://hub.docker.com/r/osrf/ros/tags

Ubuntu 22.04, ROS2 Humble, Gazebo 11 

```
docker build -t ros2_gazebo -f docker/base_ros2_gazebo.Dockerfile .
```

### overlay image
Installs and sets up gazebo and other workspace specific environments

```
docker build -t dev_overlay -f docker/worlds_overlay.Dockerfile .
```

## 