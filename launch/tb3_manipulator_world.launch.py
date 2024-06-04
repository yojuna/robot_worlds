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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
    selected_world = 'turtlebot3_world'

    world = PathJoinSubstitution(
            [
                # FindPackageShare('turtlebot3_manipulation_bringup'),
                FindPackageShare('turtlebot3_gazebo'),
                'worlds',
                selected_world+'.model'
            ]
        )

    launch_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'verbose': LaunchConfiguration('verbose'),
            'gui': LaunchConfiguration('gui'),
            'world': world
        }.items()
    )


    spawn_tb3_manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'tb3_manipulator_base.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_fake_hardware': 'true',
            'fake_sensor_commands': 'true'
            }.items()
    )

    # spawn_turtlebot_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_tb3.launch.py')
    #     ),
    #     launch_arguments={
    #         'x_pose': pose['x'],
    #         'y_pose': pose['y']
    #     }.items()
    # )
    
    spawn_tb3_manipulator = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'turtlebot3_manipulation_system',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
                ],
            output='screen',
        )
    # start rviz

    ## select rviz config
    rviz_config='tb3_world'

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
    ld.add_action(spawn_tb3_manipulator_launch)
    ld.add_action(spawn_tb3_manipulator)

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


# #!/usr/bin/env python3
# #
# # Copyright 2022 ROBOTIS CO., LTD.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
# #
# # Author: Darby Lim

# import os

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch.substitutions import PathJoinSubstitution
# from launch.substitutions import ThisLaunchFileDir

# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# # def is_valid_to_launch():
# #     # Path includes model name of Raspberry Pi series
# #     path = '/sys/firmware/devicetree/base/model'
# #     if os.path.exists(path):
# #         return False
# #     else:
# #         return True


# def generate_launch_description():
#     # if not is_valid_to_launch():
#     #     print('Can not launch fake robot in Raspberry Pi')
#     #     return LaunchDescription([])

#     start_rviz = LaunchConfiguration('start_rviz')
#     prefix = LaunchConfiguration('prefix')
#     use_sim = LaunchConfiguration('use_sim')
    
#     ## turtlebot3 world
#     ### options:
#             # empty_world
#             # turtlebot3_dqn_stage2
#             # turtlebot3_dqn_stage4  
#             # turtlebot3_world
#             # turtlebot3_dqn_stage1
#             # turtlebot3_dqn_stage3
#             # turtlebot3_house
#     selected_world = 'turtlebot3_world'

#     world = LaunchConfiguration(
#         'world',
#         default=PathJoinSubstitution(
#             [
#                 # FindPackageShare('turtlebot3_manipulation_bringup'),
#                 FindPackageShare('turtlebot3_gazebo'),
#                 'worlds',
#                 selected_world+'.model'
#             ]
#         )
#     )

#     pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
#             'y': LaunchConfiguration('y_pose', default='-0.50'),
#             'z': LaunchConfiguration('z_pose', default='0.01'),
#             'R': LaunchConfiguration('roll', default='0.00'),
#             'P': LaunchConfiguration('pitch', default='0.00'),
#             'Y': LaunchConfiguration('yaw', default='0.00')}

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'start_rviz',
#             default_value='false',
#             description='Whether execute rviz2'),

#         DeclareLaunchArgument(
#             'prefix',
#             default_value='""',
#             description='Prefix of the joint and link names'),

#         DeclareLaunchArgument(
#             'use_sim',
#             default_value='true',
#             description='Start robot in Gazebo simulation.'),

#         DeclareLaunchArgument(
#             'world',
#             default_value=world,
#             description='Directory of gazebo world file'),

#         DeclareLaunchArgument(
#             'x_pose',
#             default_value=pose['x'],
#             description='position of turtlebot3'),

#         DeclareLaunchArgument(
#             'y_pose',
#             default_value=pose['y'],
#             description='position of turtlebot3'),

#         DeclareLaunchArgument(
#             'z_pose',
#             default_value=pose['z'],
#             description='position of turtlebot3'),

#         DeclareLaunchArgument(
#             'roll',
#             default_value=pose['R'],
#             description='orientation of turtlebot3'),

#         DeclareLaunchArgument(
#             'pitch',
#             default_value=pose['P'],
#             description='orientation of turtlebot3'),

#         DeclareLaunchArgument(
#             'yaw',
#             default_value=pose['Y'],
#             description='orientation of turtlebot3'),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/base.launch.py']),
#             launch_arguments={
#                 'start_rviz': start_rviz,
#                 'prefix': prefix,
#                 'use_sim': use_sim,
#             }.items(),
#         ),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 [
#                     PathJoinSubstitution(
#                         [
#                             FindPackageShare('gazebo_ros'),
#                             'launch',
#                             'gazebo.launch.py'
#                         ]
#                     )
#                 ]
#             ),
#             launch_arguments={
#                 'verbose': 'false',
#                 'world': world,
#             }.items(),
#         ),

#         Node(
#             package='gazebo_ros',
#             executable='spawn_entity.py',
#             arguments=[
#                 '-topic', 'robot_description',
#                 '-entity', 'turtlebot3_manipulation_system',
#                 '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
#                 '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
#                 ],
#             output='screen',
#         ),
#     ])

