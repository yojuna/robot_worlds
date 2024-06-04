# ROS Gazebo Worlds Collection

Collection of Gazebo ROS Simulation environments from various sources and prepared for testing with different ROS packages. 

# setup / usage

```
mkdir -p robot_worlds_ws/src
cd robot_worlds_ws

git clone https://github.com/yojuna/robot-worlds

colcon build

source install/setup.bash
```

- index for different launch files to be included eventually. 

# Robots

## Turtlebot3 with Open Manipulator X

![turtlebot3 open manipulator](https://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/tb3_manipulation_ros2_gazebo.png)

## Turtlebot3 [Waffle, Waffle Pi, Burger]

![turtlebot3 burger empty world](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)

tb3 simulation setup guide: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

```
cd  src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ../
colcon build --symlink-install && source install/setup.bash
```

Run the tb3 world simulation

```
ros2 launch robot_worlds tb3_world.launch
```

### note:
if gazebo throws errors when using with docker:
run:
```
. /usr/share/gazebo/setup.sh
```
this sets up necessary gazebo environment variables and other things. ref: https://answers.ros.org/question/358847/cannot-launch-gzclient-on-a-launch-file-results-in-shared_ptr-assertion-error/

# Worlds


---

# Turtlebot3 Simulations

[reference - robotis turtlebot3 E manual ](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

## Gazebo House World

![gazebo house world](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

after building, run,
```
ros2 launch robot_worlds tb3_house.launch.py
```
![tb3 house gazebo](./assets/tb3_house_gz.jpg)
![tb3 house rviz](./assets/tb3_house_rviz.jpg)



## Gazebo Empty World

![gazebo empty world](https://emanual.robotis.com/assets/images/platform/turtlebot3/ros2/gazebo_world.png)

---

---

# credits / sources

This repository would not have been possible without the painstaking work of these references:

- [Turtlebot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2)

- [Gazebo models and worlds collection](https://github.com/leonhartyao/gazebo_models_worlds_collection)
    - referred sources:
        - [3DGEMS](http://data.nvision2.eecs.yorku.ca/3DGEMS/)
        - [RotorS](https://github.com/ethz-asl/rotors_simulator)
        - [TU Delft](https://github.com/tudelft/gazebo_models)
        - [ARTI-Robots](https://github.com/ARTI-Robots/gazebo_worlds)
        - [Clearpath Robotics](https://github.com/clearpathrobotics/cpr_gazebo)
        - [Fetch Robotics](https://github.com/fetchrobotics/fetch_gazebo)

- [Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps)

- [PX4 Gazebo worlds](https://dev.px4.io/v1.11_noredirect/en/simulation/gazebo_worlds.html)

- [Gazebo Small Warehouse, Bookstore and Small House worlds available for simulation | ROS Discourse](https://discourse.ros.org/t/gazebo-small-warehouse-bookstore-and-small-house-worlds-available-for-simulation/14915)


## Dataset-of-Gazebo-Worlds-Models-and-Maps
- A set of Gazebo worlds models and maps that I use for testing Navigation2
- These models are tested using Gazebo 9 and Gazebo 11.

### Usage
1- Copy the model you want to use in .gazebo/models directory.

or

- Set Gazebo model path for the worlds with models directory

2- Gazebo -> Insert -> <World_Model_Name>

or

- go to gazebo word directory and type `gazebo example.world`

Most models come with maps.
### Models:

### AWS Small House
 - `export GAZEBO_MODEL_PATH=/home/<user_name>/.gazebo/models/small_house/models/`
 - `gazebo small_house.world`
 ![Small_House](https://github.com/mlherd/gazebo_worlds_models_for_testing_navigation/blob/master/worlds/small_house/small_house.jpg?raw=true)

### AWS Office
 - `export GAZEBO_MODEL_PATH=/home/<user_name>/.gazebo/models/office/models/`
 - `gazebo office.world`
 ![Office](https://github.com/mlherd/gazebo_worlds_models_for_testing_navigation/blob/master/worlds/office/office.jpg?raw=true)
 
### AWS Bookstore
 - `export GAZEBO_MODEL_PATH=/home/<user_name>/.gazebo/models/bookstore/models/`
 - `gazebo bookstore.world`
 ![Bookstore](https://github.com/mlherd/gazebo_worlds_models_for_testing_navigation/blob/master/worlds/bookstore/bookstore.jpg?raw=true)

### AWS Hospital
 - unzip the models_part# into a dicrectory called models
 - `export GAZEBO_MODEL_PATH=/home/<user_name>/.gazebo/models/hospital/models/`
 - `gazebo hospital.world`
 - `gazebo hospital_two_floors.world`
 ![Hospital](https://github.com/mlherd/gazebo_worlds_models_maps_for_testing_navigation/blob/master/worlds/hospital/hospital.png?raw=true)
 
 #### Hospital with Two Floors
 
 ![Hospital_Two_Floors](https://github.com/mlherd/gazebo_worlds_models_maps_for_testing_navigation/blob/master/worlds/hospital/two_floor.png?raw=true)

### Custom Factory
 - `export GAZEBO_MODEL_PATH=/home/<user_name>/.gazebo/models/factory/models/`
 - `gazebo factory.model`
 ![Factory](https://github.com/mlherd/gazebo_worlds_models_maps_for_testing_navigation/blob/master/worlds/factory/factory.jpg?raw=true)

### Dynamic World
 - A world with 9 dynamic obstacles that randomly move around.
 ![Dynamic_World](https://github.com/mlherd/gazebo_worlds_models_for_testing_navigation/blob/master/worlds/dynamic_world/dynamic_room.jpg?raw=true)
 
### Random World
  - A test model generated by using my random map generation tool.
![Random World](https://github.com/mlherd/gazebo_worlds_models_for_testing_navigation/blob/master/worlds/random_world/random_world.jpg?raw=true)
 
### Robotis Turtlebot 3 World
![Turtlebot 3 World](https://github.com/mlherd/gazebo_worlds_models_for_testing_navigation/blob/master/worlds/turtlebot3_world/tb_world.jpg?raw=true)

### Experiment Rooms
  - `export GAZEBO_MODEL_PATH=/home/<user_name>/experiment_rooms/models/`
  - `cd experiment_rooms/worlds/room1`
  - `gazebo world_dynamic.model`
![10x10 rooms](https://github.com/mlherd/gazebo_worlds_models_maps_for_testing_navigation/blob/master/worlds/experiment_rooms/experiment_rooms.png?raw=true)

### 10x10 Rooms with Walls
  - Empty room models with different shapes
![10x10 rooms](https://raw.githubusercontent.com/mlherd/gazebo_worlds_models_maps_for_testing_navigation/master/worlds/room_with_walls_1/simple_rooms.png)

