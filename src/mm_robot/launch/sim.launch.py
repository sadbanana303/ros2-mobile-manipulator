import os, xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource as PLS
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    p = share('mm_robot')
    urdf = xacro.process_file(os.path.join(p, 'urdf', 'mm.urdf.xacro')).toxml()
    gz = IncludeLaunchDescription(PLS(os.path.join(share('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + os.path.join(p, 'worlds', 'lab.sdf')}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mm', '-x', LaunchConfiguration('x'), '-y', '0', '-z', '0.05'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/wheel_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[('/wheel_states', '/joint_states')])
    spawners = TimerAction(period=5.0, actions=[Node(package='controller_manager', executable='spawner',
        arguments=[c, '--controller-manager-timeout', '120'])
        for c in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']])
    return LaunchDescription([DeclareLaunchArgument('x', default_value='0'), gz, rsp, spawn, bridge, spawners])
