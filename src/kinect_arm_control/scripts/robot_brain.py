#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from collections import deque
import statistics
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except Exception:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return None

class RobotBrainNode(Node):
    def __init__(self):
        super().__init__('robot_brain_node')

        self.raw_topic_name = '/camera/target_coords'
        self.window_size = 5
        self.movement_threshold = 0.015 
        self.planning_group_name = "my_arm"

        # --- KONFIGÜRASYON ---
        moveit_config_pkg = "my_custom_arm_moveit_config"
        
        robot_description_config = load_file(moveit_config_pkg, "config/custom_robot_arm_rect.urdf.xacro")
        robot_description_semantic_config = load_file(moveit_config_pkg, "config/custom_robot_arm_rect.srdf")
        kinematics_yaml = load_yaml(moveit_config_pkg, "config/kinematics.yaml")
        
        ompl_planning_yaml = load_yaml(moveit_config_pkg, "config/ompl_planning.yaml")
        if ompl_planning_yaml is None: ompl_planning_yaml = {}
        ompl_planning_yaml['planning_plugin'] = 'ompl_interface/OMPLPlanner'
        
        controllers_yaml = load_yaml(moveit_config_pkg, "config/moveit_controllers.yaml")
        moveit_controllers = {}
        if controllers_yaml:
            if "moveit_simple_controller_manager" in controllers_yaml:
                moveit_controllers = controllers_yaml["moveit_simple_controller_manager"]
            else:
                moveit_controllers = controllers_yaml

        config_dict = {
            "robot_description": robot_description_config,
            "robot_description_semantic": robot_description_semantic_config,
            "robot_description_kinematics": kinematics_yaml,
            "planning_pipelines": {
                "pipeline_names": ["ompl"], # Sadece OMPL olsun, CHOMP'u sildim!
                "default_planning_pipeline": "ompl",
                "ompl": ompl_planning_yaml
            },
            "planning_scene_monitor_options": {
                "name": "planning_scene_monitor",
                "robot_description": "robot_description",
                "joint_state_topic": "/joint_states",
            },
            "trajectory_execution": {
                "moveit_manage_controllers": True,
                "allowed_execution_duration_scaling": 1.2,
                "allowed_goal_duration_margin": 0.5,
                "allowed_start_tolerance": 0.01
            },
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": moveit_controllers
        }

        # --- MOVEITPY BAŞLAT ---
        try:
            self.robot = MoveItPy(node_name="moveit_py", config_dict=config_dict)
            self.arm_group = self.robot.get_planning_component(self.planning_group_name)
            self.get_logger().info(f'✅ MoveItPy Hazır! Grup: {self.planning_group_name}')
        except Exception as e:
            self.get_logger().error(f'❌ MoveIt Hatası: {e}')
            self.arm_group = None
            return

        self.subscription = self.create_subscription(Point, self.raw_topic_name, self.target_callback, 10)
        self.buffer_x = deque(maxlen=self.window_size)
        self.buffer_y = deque(maxlen=self.window_size)
        self.buffer_z = deque(maxlen=self.window_size)
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0
        self.is_moving = False

    def target_callback(self, msg):
        if self.is_moving: return
        self.buffer_x.append(msg.x)
        self.buffer_y.append(msg.y)
        self.buffer_z.append(msg.z)
        
        if len(self.buffer_x) < 3: return

        median_x = statistics.median(self.buffer_x)
        median_y = statistics.median(self.buffer_y)
        median_z = statistics.median(self.buffer_z)

        diff = math.sqrt((median_x - self.current_target_x)**2 + 
                         (median_y - self.current_target_y)**2 + 
                         (median_z - self.current_target_z)**2)

        if diff > self.movement_threshold:
            self.get_logger().info(f'🎯 Hedef: X:{median_x:.2f} Y:{median_y:.2f} Z:{median_z:.2f}')
            self.current_target_x = median_x
            self.current_target_y = median_y
            self.current_target_z = median_z
            self.move_robot_to_target(median_x, median_y, median_z)

    def move_robot_to_target(self, x, y, z):
        if self.arm_group is None: return
        self.is_moving = True

        try:
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "world"
            pose_goal.pose.position.x = x
            pose_goal.pose.position.y = y
            pose_goal.pose.position.z = z
            pose_goal.pose.orientation.w = 0.0
            pose_goal.pose.orientation.x = 1.0 
            pose_goal.pose.orientation.y = 0.0
            pose_goal.pose.orientation.z = 0.0

            self.arm_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")
            self.arm_group.set_start_state_to_current_state()

            self.get_logger().info("📐 Planlanıyor (Parametreli)...")
            
            # --- KRİTİK DENEME: Parametre ile Planla ---
            # plan() fonksiyonu parametre alıyor mu diye deniyoruz.
            # Alıyorsa bu çalışacak, almıyorsa eski usul deneyeceğiz.
            try:
                plan_result = self.arm_group.plan(planning_pipeline="ompl") 
            except:
                self.get_logger().warn("⚠️ Parametreli planlama başarısız, normal deneniyor.")
                plan_result = self.arm_group.plan()
            
            if plan_result:
                self.get_logger().info("🚀 Hareket Başlıyor...")
                self.arm_group.execute()
                self.get_logger().info("🏁 Tamamlandı!")
            else:
                self.get_logger().warn("⚠️ Plan Bulunamadı!")

        except Exception as e:
            self.get_logger().error(f"Hareket Hatası: {e}")
        
        self.is_moving = False

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
