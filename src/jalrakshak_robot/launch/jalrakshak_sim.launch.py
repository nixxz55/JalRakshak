import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('jalrakshak_description')
    
    # Path to pipe world and URDF
    world_file = os.path.join(pkg_description, 'worlds', 'pipe_env.world')
    urdf_file = os.path.join(pkg_description, 'urdf', 'jalrakshak.urdf')
    
    # Direct Gazebo process with world file
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn the robot inside the pipe
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'jalrakshak', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )
    
    # Pressure monitoring node
    pressure_node = Node(
        package='jalrakshak_robot',
        executable='pressure_node',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity,
        pressure_node
    ])
