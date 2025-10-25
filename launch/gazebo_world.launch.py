#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='titanfall_ai').find('titanfall_ai')
    worlds_dir = os.path.join(pkg_share, 'worlds')
    models_dir = os.path.join(pkg_share, 'models')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='test_environment.world')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='test_environment.world',
        description='Full path to world file to load')
    
    # Launch Gazebo
    gazebo_node = Node(
        package='gazebo_ros',
        executable='gazebo',
        name='gazebo',
        output='screen',
        arguments=[
            os.path.join(worlds_dir, world_file),
            '--verbose'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Spawn robot model
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-file', os.path.join(models_dir, 'exosuit_model.sdf'),
            '-entity', 'titanfall_exosuit',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    
    # Add the actions to launch Gazebo and spawn the robot
    ld.add_action(gazebo_node)
    ld.add_action(spawn_robot)
    
    return ld
