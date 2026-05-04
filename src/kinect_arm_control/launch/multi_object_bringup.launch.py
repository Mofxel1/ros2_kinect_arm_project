import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "kinect_arm_control"
    pkg_share = get_package_share_directory(pkg_name)

    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "start_system.launch.py")
        )
    )

    fix_time_sync = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/controller_manager/switch_controllers",
                    "controller_manager_msgs/srv/SwitchControllers",
                    '{deactivate_controllers: ["joint_state_broadcaster"], activate_controllers: ["joint_state_broadcaster"], strictness: 1}',
                ],
                output="screen",
            )
        ],
    )

    start_brain = TimerAction(
        period=17.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "debug_brain_queue.launch.py")
                )
            )
        ],
    )

    start_detector = TimerAction(
        period=20.0,
        actions=[
            Node(
                package=pkg_name,
                executable="detector_yolo_multi.py",
                name="detector_yolo_multi",
                output="screen",
            )
        ],
    )

    start_task_manager = TimerAction(
        period=22.0,
        actions=[
            Node(
                package=pkg_name,
                executable="task_manager_node.py",
                name="task_manager_node",
                output="screen",
            )
        ],
    )

    # Sepet collision objeleri istersen aktif.
    # Şimdilik gripper olmadığı için sadece RViz/MoveIt sahnesinde görünmesi için çalıştırıyoruz.
    start_obstacles = TimerAction(
        period=24.0,
        actions=[
            Node(
                package=pkg_name,
                executable="add_obstacle.py",
                name="obstacle_spawner",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        start_simulation,
        fix_time_sync,
        start_brain,
        start_detector,
        start_task_manager,
        start_obstacles,
    ])