import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. URDF DOSYASININ KONUMUNU BELİRLE
    # Senin verdiğin yol: kinect_arm_control/description/urdf/my_custom_robot.urdf
    # NOT: Dosya adının "my_custom_robot.urdf" olduğunu varsayıyorum (yüklediğin dosya buydu).
    # Eğer dosya adı farklıysa (örn. robot.urdf.xacro) aşağıdan değiştir.
    pkg_share = get_package_share_directory('kinect_arm_control')
    urdf_file_path = os.path.join(pkg_share, "description", "urdf", "my_custom_robot.urdf")

    # Dosyanın varlığını kontrol et (Debug kolaylığı için)
    if not os.path.exists(urdf_file_path):
        print(f"\n\nUYARI: URDF dosyası bulunamadı: {urdf_file_path}")
        print("Lütfen CMakeLists.txt dosyasina 'install(DIRECTORY description...)' eklediğinden emin ol!\n\n")

    # 2. MOVEIT CONFIG'İ YÜKLE (URDF Yolu ile Birlikte)
    # .robot_description(file_path=...) diyerek varsayılanı eziyoruz ve senin dosyanı gösteriyoruz.
    moveit_config = MoveItConfigsBuilder("my_custom_arm_moveit_config", package_name="my_custom_arm_moveit_config") \
        .robot_description(file_path=urdf_file_path) \
        .to_moveit_configs()

    # 3. BRAIN NODE
    cpp_brain_node = Node(
        package="kinect_arm_control",
        executable="cpp_brain",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True} 
        ]
    )

    return LaunchDescription([
        cpp_brain_node
    ])
