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
    launch_dir = os.path.join(pkg_share, 'launch')
    config_dir = os.path.join(pkg_share, 'config')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='test_environment.world')
    robot_model = LaunchConfiguration('robot_model', default='exosuit_model.sdf')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='test_environment.world',
        description='Full path to world file to load')
    
    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='exosuit_model.sdf',
        description='Full path to robot model file to load')
    
    # Include Gazebo world launch
    gazebo_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo_world.launch.py')
        ),
        launch_arguments={
            'world_file': world_file,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Launch the main Titan Fall AI node
    titanfall_ai_node = Node(
        package='titanfall_ai',
        executable='titanfall_ai_node',
        name='titanfall_ai_main',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(config_dir, 'ros_topics.yaml'),
            os.path.join(config_dir, 'pid_params.yaml'),
            os.path.join(config_dir, 'system_thresholds.yaml'),
            os.path.join(config_dir, 'power_limits.yaml')
        ],
        remappings=[
            ('/imu_data', '/imu'),
            ('/joint_states', '/joint_states'),
            ('/joint_torques', '/joint_torques'),
            ('/battery_voltage', '/battery_voltage'),
            ('/temperature_data', '/temperature')
        ]
    )
    
    # Launch joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(config_dir, 'ros_topics.yaml')
        ]
    )
    
    # Launch joint trajectory controller
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_trajectory_controller_spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Launch joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Launch RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'titanfall_ai.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_robot_model_cmd)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(gazebo_world_launch)
    ld.add_action(titanfall_ai_node)
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(joint_trajectory_controller)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(rviz_node)
    
    return ld
