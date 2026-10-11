from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    mc = MoveItConfigsBuilder("mm", package_name="mm_moveit_config").to_moveit_configs()
    mg = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
              parameters=[mc.to_dict(), {"use_sim_time": True}])
    rviz = Node(package="rviz2", executable="rviz2", output="log",
                arguments=["-d", str(mc.package_path / "config/moveit.rviz")],
                parameters=[mc.robot_description, mc.robot_description_semantic,
                            mc.robot_description_kinematics, mc.planning_pipelines,
                            mc.joint_limits, {"use_sim_time": True}])
    return LaunchDescription([mg, rviz])
