import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction

def generate_launch_description():

    package_description = "lapy_bot"
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_config_file = os.path.join(get_package_share_directory(package_description),
                                    'config', 'mapper_params_online_async.yaml')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py'
        )]), launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_config_file}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        slam_toolbox_launch
    ])