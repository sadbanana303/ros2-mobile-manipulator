import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource as PLS
from launch_ros.actions import Node

def generate_launch_description():
    p, n = share('mm_robot'), share('nav2_bringup')
    nav = IncludeLaunchDescription(PLS(os.path.join(n, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': os.path.join(p, 'maps', 'lab.yaml'),
                          'params_file': os.path.join(p, 'config', 'nav2_params.yaml'),
                          'use_sim_time': 'true', 'use_composition': 'False'}.items())
    relay = Node(package='topic_tools', executable='relay', arguments=['/cmd_vel_nav', '/cmd_vel'])
    return LaunchDescription([nav, relay])
