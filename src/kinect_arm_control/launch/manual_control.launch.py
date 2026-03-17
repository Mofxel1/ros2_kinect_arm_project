import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('kinect_arm_control')

    # 1. SİMÜLASYONU BAŞLAT (Gazebo, RViz, MoveIt, Motor Sürücüleri)
    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'start_system.launch.py')
        )
    )

    # 2. ZAMAN SENKRONİZASYONU TAMİRİ 
    # Simülasyon açıldıktan 15 saniye sonra çalışır.
    fix_time_sync = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', 
                    '/controller_manager/switch_controllers', 
                    'controller_manager_msgs/srv/SwitchControllers',
                    '{deactivate_controllers: ["joint_state_broadcaster"], activate_controllers: ["joint_state_broadcaster"], strictness: 1}'
                ],
                output='screen'
            )
        ]
    )

    # 3. TERMİNAL KONTROLCÜSÜNÜ BAŞLAT
    # Fix yapıldıktan 3 saniye sonra (Toplam 18. saniye) başlasın.
    # prefix='gnome-terminal --' sayesinde Gazebo loglarına karışmadan YEPYENİ bir pencerede açılacak!
    start_terminal = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='kinect_arm_control',
                executable='teleop_terminal.py',
                name='teleop_terminal',
                output='screen',
                prefix='gnome-terminal --' 
            )
        ]
    )

    return LaunchDescription([
        start_simulation,
        fix_time_sync,
        start_terminal
    ])
