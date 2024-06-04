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
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# export TURTLEBOT3_MODEL=waffle

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
    robot_worlds_dir = get_package_share_directory('robot_worlds')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_gazebo_verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true to enable verbose mode for Gazebo.')
    declare_gazebo_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')
    

    # world = os.path.join(
    #     get_package_share_directory('robot_worlds'),
    #     'worlds',
    #     'turtlebot3_house'
    #     'turtlebot3_house.world'
    # )

    # world = get_robot_world('office', 'service')

    ## turtlebot3 world
    ### options:
            # empty_world
            # turtlebot3_dqn_stage2
            # turtlebot3_dqn_stage4  
            # turtlebot3_world
            # turtlebot3_dqn_stage1
            # turtlebot3_dqn_stage3
            # turtlebot3_house

    SELECTED_WORLD = 'turtlebot3_house'

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        SELECTED_WORLD+'.world'
    )
    # gz_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={
    #         'verbose': 'true',
    #         'world': world,
    #     }.items(),
    # )

    launch_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'verbose': LaunchConfiguration('verbose'),
            'gui': LaunchConfiguration('gui'),
            'world': world
        }.items()
    )


    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'tb3_robot_state_publisher.launch.py')
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

    # start rviz

    ## select rviz config
    rviz_config='tb3_house'

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_enabled',
        default_value='true',
        description='Set to true to launch rviz.')
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            robot_worlds_dir, 'rviz', rviz_config+'.rviz'),
        description='Define rviz config file to be used.')

    launch_rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz_enabled')),
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

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




    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    # ld.add_action(gz_launch)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    # gazebo
    ld.add_action(declare_gazebo_verbose_arg)
    ld.add_action(declare_gazebo_gui_arg)
    ld.add_action(launch_gazebo_world)

    # # rviz
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_rviz_config_arg)
    ld.add_action(launch_rviz)

    # teleop
    ld.add_action(declare_teleop_arg)
    ld.add_action(launch_teleop)


    return ld





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