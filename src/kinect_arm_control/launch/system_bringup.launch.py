import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    pkg_name = 'kinect_arm_control'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. SİMÜLASYONU BAŞLAT (Start System)
    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'start_system.launch.py')
        )
    )

    # 2. ZAMAN SENKRONİZASYONU TAMİRİ (Sihirli Komut)
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

    # 3. BEYNİ BAŞLAT (MoveIt + IK Solver)
    # Fix yapıldıktan 2 saniye sonra başlasın (Toplam 17. saniye)
    start_brain = TimerAction(
        period=17.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'debug_brain.launch.py')
                )
            )
        ]
    )

    # 4. GÖZÜ BAŞLAT (YOLO)
    # En son bu başlasın (Toplam 20. saniye)
    start_yolo = TimerAction(
        period=20.0,
        actions=[
            Node(
                package=pkg_name,
                executable='detector_yolo.py',
                name='detector_yolo',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        start_simulation,
        fix_time_sync,
        start_brain,
        start_yolo
    ])
