"""Sample launch file"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    # Replace with your project files
    project_gazebo_path = get_package_share_directory('sim_fiducial')
    world_path = os.path.join(project_gazebo_path, 'worlds', 'one_marker.world')
    sdf_path = os.path.join(project_gazebo_path, 'sdf', 'forward_camera.sdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_path
        ], output='screen'),

        # Add the robot SDF or URDF to the simulation
        Node(package='sim_fiducial', node_executable='inject_entity.py', output='screen',
             arguments=[sdf_path, '0', '0', '0', '0', '0', '0']),

        # If required, publish static joints -- only works with URDF files
        # Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
        #     node_name='robot_state_publisher', arguments=[sdf_path]),

    ])
