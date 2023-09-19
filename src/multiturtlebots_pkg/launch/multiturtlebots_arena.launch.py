# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""
Demo for spawn_entity.
Launches Gazebo and spawns a model
"""
# A bunch of software packages that are needed to launch ROS2
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'multiturtlebotsBC_arena.world'
    pkg_dir = get_package_share_directory('multiturtlebots_pkg')

    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    #GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    #spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
    #                    arguments=['-entity', 'demo', 'x', 'y', 'z'],
    #                    output='screen')
    spawn_entity1 = Node(package='multiturtlebots_pkg', executable='spawn_turtlebot3',
                        arguments=['turtlebot_1', 'bot1', '3.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1'],
                        output='screen')

    spawn_entity2 = Node(package='multiturtlebots_pkg', executable='spawn_turtlebot3',
                        arguments=['turtlebot_2', 'bot2', '1.5', '1.5', '0.0', '0.0', '0.0', '0.3826834', '0.9238795'],
                        output='screen')

    spawn_entity3 = Node(package='multiturtlebots_pkg', executable='spawn_turtlebot3',
                        arguments=['turtlebot_3', 'bot3', '0.0', '3.0', '0.0', '0.0', '0.0', '0.7071068', '0.7071068'],
                        output='screen')

    spawn_entity4 = Node(package='multiturtlebots_pkg', executable='spawn_turtlebot3',
                        arguments=['turtlebot_4', 'bot4', '-1.5', '1.5', '0.0', '0.0', '0.0', '0.9238795', '0.3826834'],
                        output='screen')
                        
    spawn_entity5 = Node(package='multiturtlebots_pkg', executable='spawn_turtlebot3',
                        arguments=['turtlebot_5', 'bot5', '-3.0', '0.0', '0.0', '0.0', '0.0', '1', '0'],
                        output='screen')
                        
    spawn_entity6 = Node(package='multiturtlebots_pkg', executable='spawn_turtlebot3',
                        arguments=['turtlebot_6', 'bot6', '-1.5', '-1.5', '0.0', '0.0', '0.0', '0.9238795', '-0.3826834'],
                        output='screen')
                        
    spawn_entity7 = Node(package='multiturtlebots_pkg', executable='spawn_turtlebot3',
                        arguments=['turtlebot_7', 'bot7', '0.0', '-3.0', '0.0', '0.0', '0.0', '0.7071068', '-0.7071068'],
                        output='screen')
                        
    spawn_entity8 = Node(package='multiturtlebots_pkg', executable='spawn_turtlebot3',
                        arguments=['turtlebot_8', 'bot8', '1.5', '-1.5', '0.0', '0.0', '0.0', '0.3826834', '-0.9238795'],
                        output='screen')  
                                                                                                                        

    return LaunchDescription([
        gazebo,
        spawn_entity1,
        spawn_entity2,
        spawn_entity3,
        spawn_entity4,
        spawn_entity5,
        spawn_entity6,
        spawn_entity7,
        spawn_entity8,
    ])

