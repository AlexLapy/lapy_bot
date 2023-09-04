import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    package_name='lapy_mapper'
    use_sim_time = LaunchConfiguration('use_sim_time')

    cartographer_config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    configuration_basename = "cartographer.lua"


    occu_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}],
    )

    cartographer_node = Node(            
        package='cartographer_ros', 
        executable='cartographer_node', 
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        remappings=[('/odom', '/diff_drive_controller/odom')]
        )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        cartographer_node,
        occu_grid_node,
    ])
