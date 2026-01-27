import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return {} # Hata verirse boş dön, çökme.

def generate_launch_description():
    moveit_config_pkg = "my_custom_arm_moveit_config"
    moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg)
    
    xacro_file_path = os.path.join(moveit_config_pkg_share, "config", "custom_robot_arm_rect.urdf.xacro")
    
    moveit_config_builder = MoveItConfigsBuilder(moveit_config_pkg, package_name=moveit_config_pkg) \
        .robot_description(file_path=xacro_file_path)
    
    moveit_config = moveit_config_builder.to_moveit_configs()

    # --- CHOMP AYARLARI (Manuel) ---
    chomp_planning_config = {
        "planning_plugin": "chomp_interface/CHOMPPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
        "jiggle_fraction": 0.05,
        "planning_time_limit": 10.0,
        "max_iterations": 200,
        "max_recovery_attempts": 1,
        "smoothness_cost_weight": 0.1,
        "obstacle_cost_weight": 1.0,
        "learning_rate": 0.01,
        "use_stochastic_descent": True,
        "enable_failure_recovery": True,
        "max_planning_threads": 4,
        "trajectory_initialization_method": "quintic-spline"
    }

    ompl_planning_yaml = load_yaml(moveit_config_pkg, "config/ompl_planning.yaml")

    planning_pipelines_config = {
        "planning_pipelines": ["chomp", "ompl"],
        "default_planning_pipeline": "chomp", # Varsayılan CHOMP
        "chomp": chomp_planning_config,
        "ompl": ompl_planning_yaml
    }
    
    # Controller Config Dosyasını Elle Gösteriyoruz (Çok Önemli!)
    controller_config = os.path.join(moveit_config_pkg_share, "config", "ros2_controllers.yaml")

    # 1. Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 2. Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 3. Move Group
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

    # 4. RViz
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

    # 5. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {"use_sim_time": True}]
    )

    # 6. Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Parametre dosyasını yüklüyoruz!
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_arm_controller", "--controller-manager", "/controller_manager", "--param-file", controller_config],
    )

    # 7. C++ Beyin
    cpp_brain_node = Node(
        package="kinect_arm_control",
        executable="cpp_brain",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            planning_pipelines_config,
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
        spawn_entity,
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
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[cpp_brain_node],
            )
        ),
    ])
