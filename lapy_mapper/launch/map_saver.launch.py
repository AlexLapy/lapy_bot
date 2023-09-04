import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


# DOESNT WORK, only use default
def generate_launch_description():

    package_name = "lapy_mapper"
    file_dir = os.path.join(get_package_share_directory(package_name), 'maps')
    #file_name = LaunchConfiguration('file_name')
    #string_file_name = LaunchConfiguration('string_file_name')

    declare_file_name_cmd = DeclareLaunchArgument(
            'file_name',
            default_value='default_map',
            description='File name without ext.')
    
    declare_string_file_name_cmd = DeclareLaunchArgument(
        'string_file_name', default_value=[LaunchConfiguration('file_name')]
    )

    map_saver_node = Node(
            package='nav2_map_server',
            node_executable='map_saver_cli',
            output='screen',
            arguments=['-f', os.path.join(file_dir, 'default_map')],
    )

    return LaunchDescription([
        declare_file_name_cmd,
        declare_string_file_name_cmd,

        map_saver_node,
        ])
