import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return {}

def generate_launch_description():
    moveit_config_pkg = "my_custom_arm_moveit_config"
    moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg)
    
    xacro_file_path = os.path.join(moveit_config_pkg_share, "config", "custom_robot_arm_rect.urdf.xacro")
    
    moveit_config_builder = MoveItConfigsBuilder(moveit_config_pkg, package_name=moveit_config_pkg) \
        .robot_description(file_path=xacro_file_path)
    
    moveit_config = moveit_config_builder.to_moveit_configs()

    # --- CHOMP & PLANNING CONFIG ---
    chomp_planning_config = {
        "planning_plugin": "chomp_interface/CHOMPPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
        "jiggle_fraction": 0.05,
        "planning_time_limit": 10.0,
        "max_iterations": 200,
        "trajectory_initialization_method": "quintic-spline"
    }

    ompl_planning_yaml = load_yaml(moveit_config_pkg, "config/ompl_planning.yaml")

    planning_pipelines_config = {
        "planning_pipelines": ["chomp", "ompl"],
        "default_planning_pipeline": "chomp",
        "chomp": chomp_planning_config,
        "ompl": ompl_planning_yaml
    }
    
    controller_config = os.path.join(moveit_config_pkg_share, "config", "ros2_controllers.yaml")

    # --- 1. KINECT SÜRÜCÜSÜ ---
    # DEBUG MODU: Manuel başlatacağımız için kapattık.
    # kinect_driver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('kinect_ros2'), 'launch', 'kinect_ros2.launch.py')
    #     ])
    # )

    # --- 2. STATIC TRANSFORM ---
    # Kamera ile Robot arasındaki fiziksel bağlantı (Görselleştirme için)
    # Eğer kamerayı robotun arkasına koyduysan buradaki değerleri güncellemelisin.
    # Örn: Robotun 50cm arkası (-0.5), 0.5m yukarısı
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect_to_robot_tf',
        arguments=['-0.5', '0.0', '0.5', '0.0', '0.0', '0.0', 'base_link', 'kinect_link']
    )

    # 3. Gazebo (Simülasyon Ortamı)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 4. Robotu Simülasyona Ekleme
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 5. Move Group (Hareket Planlayıcı)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(), 
            planning_pipelines_config, 
            {"use_sim_time": True}
        ],
    )

    # 6. RViz (Görselleştirme)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(moveit_config_pkg_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            planning_pipelines_config,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ]
    )

    # 7. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {"use_sim_time": True}]
    )

    # 8. Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_arm_controller", "--controller-manager", "/controller_manager", "--param-file", controller_config],
    )

    # 9. C++ Beyin
    # DEBUG MODU: Manuel başlatacağımız için kapattık (ros2 run kinect_arm_control cpp_brain)
    # cpp_brain_node = Node(
    #     package="kinect_arm_control",
    #     executable="cpp_brain",
    #     output="screen",
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         planning_pipelines_config,
    #         {"use_sim_time": True}
    #     ]
    # )

    # 10. Eye Node (YOLO)
    # DEBUG MODU: Manuel başlatacağımız için kapattık (python3 detector_yolo.py)
    # eye_node = Node(
    #     package="kinect_arm_control",
    #     executable="detector_yolo.py", # İsim düzeltildi
    #     output="screen",
    #     parameters=[{"use_sim_time": True}]
    # )

    return LaunchDescription([
        static_tf,
        # kinect_driver, # Manuel
        gazebo,
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
        spawn_entity,
        # eye_node,      # Manuel
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        # cpp_brain_node'u da manuel başlatacağız, o yüzden buradan kaldırdık
    ])
