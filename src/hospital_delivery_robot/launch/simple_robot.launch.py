
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_hospital_delivery = get_package_share_directory('hospital_delivery_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    urdf_file = os.path.join(pkg_hospital_delivery, 'urdf', 'hospital_robot_simple.urdf')
    
    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )
    
    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'hospital_robot',
                   '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
