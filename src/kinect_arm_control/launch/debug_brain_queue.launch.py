import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("kinect_arm_control")
    urdf_file_path = os.path.join(pkg_share, "description", "urdf", "robot_montaj.urdf")

    if not os.path.exists(urdf_file_path):
        print(f"\n\nUYARI: URDF dosyası bulunamadı: {urdf_file_path}\n\n")

    moveit_config = (
        MoveItConfigsBuilder(
            "my_custom_arm_moveit_config",
            package_name="my_custom_arm_moveit_config"
        )
        .robot_description(file_path=urdf_file_path)
        .to_moveit_configs()
    )

    cpp_brain_node = Node(
        package="kinect_arm_control",
        executable="dynamic_brain_node_queue",
        name="dynamic_brain_node_queue",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        cpp_brain_node
    ])