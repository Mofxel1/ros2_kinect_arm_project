import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Paket Yollarını Tanımla ---
    pkg_kinect_arm_control = get_package_share_directory('kinect_arm_control')
    pkg_moveit_config = get_package_share_directory('my_custom_arm_moveit_config')
    pkg_kinect_ros2 = get_package_share_directory('kinect_ros2')

    # --- 2. Argümanlar ---
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo', default_value='true',
        description='Gazebo simülasyonunu başlatır'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Simülasyon zamanı kullanılsın mı?'
    )

    # --- 3. Başlatma Tanımları ---

    # A) GAZEBO VE ROBOT
    # Not: Bu dosya genellikle kendi içinde RViz'i de tetikler.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'gazebo.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_gazebo'))
    )

    # B) MOVEIT (Move Group) - Planlama Motoru
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # (İPTAL EDİLDİ) C) RVIZ
    # Zaten gazebo.launch.py içinde veya MoveIt config içinde açıldığı için
    # buradaki manuel başlatmayı kaldırdık.

    # D) KINECT KAMERA
    kinect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_kinect_ros2, 'launch', 'pointcloud.launch.py')
        )
    )

    # E) SENİN KODLARIN
    config_file = os.path.join(pkg_kinect_arm_control, 'config', 'params.yaml')

    # 1. YOLO Algılayıcı
    detector_node = Node(
        package='kinect_arm_control',
        executable='detector_yolo',
        name='yolo_detector',
        output='screen',
        parameters=[config_file]
    )

    # 2. Robot Beyni
    brain_node = Node(
        package='kinect_arm_control',
        executable='brain',
        name='robot_brain_manager',
        output='screen',
        parameters=[config_file]
    )

    # --- 4. ZAMANLAMA VE SIRALAMA ---
    return LaunchDescription([
        use_gazebo_arg,
        use_sim_time_arg,
        
        # 1. Aşama: Ortam (Gazebo+RViz bu paketin içinden gelir, Kinect)
        gazebo_launch,
        kinect_launch,

        # 2. Aşama: Planlama (Gazebo açıldıktan 5 sn sonra MoveIt başlasın)
        TimerAction(
            period=5.0,
            actions=[move_group_launch]
        ),

        # (RViz Timer'ı kaldırıldı, Gazebo ile birlikte otomatik açılacak)

        # 3. Aşama: Senin Kodların (Sistem oturduktan 10 sn sonra beyin devreye girsin)
        TimerAction(
            period=10.0,
            actions=[
                detector_node,
                brain_node
            ]
        )
    ])
