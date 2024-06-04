#!/usr/bin/env python3
#
# Copyright 2020 ROBOTIS CO., LTD.
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
# Authors: Hye-jong KIM

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ld = LaunchDescription()
    robot_worlds_launch_dir = os.path.join(
        get_package_share_directory(
            'robot_worlds'), 'launch')
    launch_dir = os.path.join(
        get_package_share_directory(
            'turtlebot3_manipulation_moveit_config'), 'launch')
    bringup_launch_dir = os.path.join(
        get_package_share_directory(
            'turtlebot3_manipulation_bringup'), 'launch')

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_worlds_launch_dir, '/tb3_om_rviz_moveit.launch.py'])
    )
    ld.add_action(rviz_launch)

    # move_group
    move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py']),
            launch_arguments={
                'use_sim': 'true',
            }.items(),
        )
    ld.add_action(move_group_launch)

    # gazebo_control with robot_state_publisher
    rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether execute rviz2')
    ld.add_action(rviz_arg)

    # empty_world_path = PathJoinSubstitution(
    #     [
    #         FindPackageShare('turtlebot3_manipulation_bringup'),
    #         'worlds',
    #         'turtlebot3_world.model'
    #     ]
    # )

    SELECTED_WORLD = 'turtlebot3_house'

    # world = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'worlds',
    #     SELECTED_WORLD+'.world'
    # )
    tb3_house_world_path = PathJoinSubstitution(
        [
            FindPackageShare('turtlebot3_gazebo'),
            'worlds',
            SELECTED_WORLD+'.world'
        ]
    )


    gazebo_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/gazebo.launch.py']),
        launch_arguments={
            'world': tb3_house_world_path,
            'x_pose': '2.0',
            'y_pose': '0.5',
            'z_pose': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '3.14',
        }.items(),
        )
    ld.add_action(gazebo_control_launch)

    declare_teleop_arg = DeclareLaunchArgument(
        'teleop_enabled',
        default_value='true',
        description='Set to true to enable teleop to manually move MiR around.')

    launch_teleop = Node(
        condition=IfCondition(LaunchConfiguration("teleop_enabled")),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e')
    ld.add_action(declare_teleop_arg)
    ld.add_action(launch_teleop)



    return ld
