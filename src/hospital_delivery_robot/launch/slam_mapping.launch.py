import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package directories
    pkg_hospital = get_package_share_directory('hospital_delivery_robot')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Paths
    slam_params_file = os.path.join(pkg_hospital, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(pkg_hospital, 'rviz', 'slam_mapping.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        slam_toolbox_node,
        rviz_node
    ])
