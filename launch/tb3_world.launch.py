#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def get_robot_world(world_name, launch_name):
    world = os.path.join(
        get_package_share_directory('robot_worlds'),
        'worlds',
        world_name,
        launch_name+".world"
    )
    return world    


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('robot_worlds'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # x_pose = LaunchConfiguration('x_pose', default='-2.0')
    # y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # world = os.path.join(
    #     get_package_share_directory('robot_worlds'),
    #     'worlds',
    #     'turtlebot3_house'
    #     'turtlebot3_house.world'
    # )

    world = get_robot_world('office', 'service')

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'world': world,
        }.items(),
    )

    # gz_node = IncludeLaunchDescription(
    #     Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         arguments=[
    #             '-topic', 'robot_description',
    #             '-entity', 'turtlebot3_manipulation_system',
    #             '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
    #             '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
    #             ],
    #         output='screen',
    #     )
    # )

    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items()
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pose': pose['x'],
            'y_pose': pose['y']
        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    ld.add_action(gz_launch)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld