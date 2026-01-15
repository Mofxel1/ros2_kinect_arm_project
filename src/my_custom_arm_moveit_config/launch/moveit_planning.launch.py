import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    pkg_path = get_package_share_directory('my_custom_arm_moveit_config')
    
    # 1. URDF
    xacro_file = os.path.join(pkg_path, 'config', 'custom_robot_arm_rect.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # 2. SRDF
    srdf_file = os.path.join(pkg_path, 'config', 'custom_robot_arm_rect.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # 3. Ayar Dosyaları
    kinematics_yaml = load_yaml('my_custom_arm_moveit_config', 'config/kinematics.yaml')
    moveit_controllers = load_yaml('my_custom_arm_moveit_config', 'config/moveit_controllers.yaml')
    
    # --- YENİ EKLENEN KISIM: OMPL AYARLARI ---
    ompl_planning_yaml = load_yaml('my_custom_arm_moveit_config', 'config/ompl_planning.yaml')
    
    planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    # ----------------------------------------

    # 4. MoveGroup Node
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            moveit_controllers,
            planning_pipeline_config, # OMPL ayarlarını buraya verdik
            {'moveit_manage_controllers': True},
            {'trajectory_execution.allowed_execution_duration_scaling': 5.0},
            {'trajectory_execution.allowed_goal_duration_margin': 0.5},
            {'trajectory_execution.allowed_start_tolerance': 0.05}, 
            {'use_sim_time': True},
        ],
    )

    # 5. RViz Node
    rviz_config_file = os.path.join(pkg_path, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipeline_config,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([run_move_group_node, rviz_node])
