# tasks

## TODO:
    - index and restructure directories for all robot and world models 
    - add description, specific credits, usage info and images for each world & robot
    - add specific launch files for all worlds
    - add launch files with configurable launch arguments for different robots and worlds
    - add nav2 library, interfaced modularly with repository

# directory structure

## index
(ros2) nemo@homelab:~/code/repos/roboforge_dev/robot_worlds_ws$ tree -L 1 src/robot_worlds/worlds/
src/robot_worlds/worlds/
├── bookstore
├── dynamic_obstacle
├── dynamic_world
├── empty_room
├── experiment_rooms
├── factory
├── hospital
├── office
├── random_world
├── room_with_walls_1
├── room_with_walls_2
├── small_house
├── star_room_with_walls
├── turtlebot3_dqn_world
├── turtlebot3_house
└── turtlebot3_world

16 directories, 0 files


## directories and files

```
(ros2) nemo@homelab:~/code/repos/roboforge_dev/robot_worlds_ws$ tree -L 2 src/robot_worlds/worlds/
src/robot_worlds/worlds/
├── bookstore
│   ├── bookstore.jpg
│   ├── bookstore.world
│   ├── bookstore.zip
│   └── models
├── dynamic_obstacle
│   ├── model.config
│   └── model.sdf
├── dynamic_world
│   ├── dynamic_room.jpg
│   └── world.model
├── empty_room
│   ├── maps
│   ├── model.config
│   └── model.sdf
├── experiment_rooms
│   ├── experiment_rooms.png
│   ├── models
│   ├── pics
│   ├── README.md
│   └── worlds
├── factory
│   ├── factory.jpg
│   ├── factory.model
│   └── models
├── hospital
│   ├── hospital.png
│   ├── hospital_two_floors.world
│   ├── hospital.world
│   ├── models
│   ├── models_part1.zip
│   ├── models_part2.zip
│   ├── models_part3.zip
│   ├── models_part4.zip
│   ├── photos
│   ├── two_floor.png
│   └── unzip_models.sh
├── office
│   ├── map
│   ├── media
│   ├── models
│   ├── office.jpg
│   ├── office_part1.zip
│   ├── office_part2.zip
│   └── service.world
├── random_world
│   ├── media
│   ├── model.config
│   ├── model.sdf
│   └── random_world.jpg
├── room_with_walls_1
│   ├── map_0.05m-px.png
│   ├── map_0.1m-px.png
│   ├── map.yaml
│   ├── room_with_walls_10x10
│   └── simple_rooms.png
├── room_with_walls_2
│   ├── map_0.05m-px.png
│   ├── map_0.1m-px.png
│   ├── map.yaml
│   └── room3
├── small_house
│   ├── maps
│   ├── models
│   ├── photos
│   ├── small_house
│   ├── small_house.jpg
│   ├── small_house.world
│   └── small_house.zip
├── star_room_with_walls
│   ├── map_0.05m-px.png
│   ├── map_0.1m-px.png
│   ├── map.yaml
│   └── room_with_walls_star
├── turtlebot3_dqn_world
│   ├── goal_box
│   ├── inner_walls
│   ├── model.config
│   ├── model.sdf
│   ├── obstacle1
│   ├── obstacle2
│   ├── obstacle_plugin
│   ├── obstacles
│   ├── turtlebot3_dqn_stage1.world
│   ├── turtlebot3_dqn_stage2.world
│   ├── turtlebot3_dqn_stage3.world
│   └── turtlebot3_dqn_stage4.world
├── turtlebot3_house
│   ├── model.config
│   ├── model.sdf
│   └── turtlebot3_house.world
└── turtlebot3_world
    ├── empty_world.world
    ├── maps
    ├── meshes
    ├── model-1_4.sdf
    ├── model.config
    ├── model.sdf
    ├── tb_world.jpg
    └── turtlebot3_world.world

43 directories, 57 files

```
