import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    package_navigation = "lapy_navigation"

    config = os.path.join(get_package_share_directory(package_navigation),'config','spot_list.yaml')

    return LaunchDescription([ 
            
        Node(
            package = package_navigation,
            executable = 'go_to_pose_from_joy',
            name = 'move_to_spot_from_joy',
            parameters = [
                {'use_sim_time': False},
                config
            ],
            output = 'screen',
        )
    ])