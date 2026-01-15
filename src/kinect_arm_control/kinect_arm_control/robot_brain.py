import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class RobotBrain(Node):
    def __init__(self):
        super().__init__('robot_brain_node')

        # --- AYARLAR ---
        self.movement_threshold = 0.015  # 1.5 cm (Bundan az oynarsa gitmez)
        
        # Robot Kol Uzunlukları
        self.L1, self.L2, self.L3 = 0.215, 0.230, 0.200
        
        # Kamera Konumu (Robot Tabanına Göre)
        self.cam_offset_x = 0.70
        self.cam_height = 0.30
        
        # Kamera Kalibrasyon Değerleri (Kinect)
        self.fx, self.fy = 594.21, 591.04
        self.cx, self.cy = 320.0, 240.0
        
        self.last_target_xyz = [0.0, 0.0, 0.0] 
        
        # --- BAĞLANTILAR ---
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("MoveIt (Beyin) araniyor...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveIt Baglandi! Robot hazir.")

        self.create_subscription(Image, '/image_raw', self.rgb_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Image, '/depth/image_raw', self.depth_callback, qos_profile=qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.latest_depth_img = None
        self.robot_is_busy = False 

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except:
            pass

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        # Yeşil Top Tespiti
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([40, 100, 70]), np.array([80, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        status_text = "NESNE ARANIYOR..."
        status_color = (0, 255, 255) # Sarı

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((u, v), radius) = cv2.minEnclosingCircle(c)
            
            if radius > 10:
                cv2.circle(frame, (int(u), int(v)), int(radius), (0, 255, 0), 2)
                
                if self.robot_is_busy:
                    status_text = "HAREKET EDIYOR..."
                    status_color = (0, 0, 255) # Kırmızı
                elif self.latest_depth_img is not None:
                    u, v = int(u), int(v)
                    try:
                        if v < self.latest_depth_img.shape[0] and u < self.latest_depth_img.shape[1]:
                            raw_depth = self.latest_depth_img[v, u]
                            if raw_depth > 0:
                                # Hesaplamalar
                                Z_cam = raw_depth / 1000.0 if raw_depth > 100 else raw_depth
                                X_cam = (u - self.cx) * Z_cam / self.fx
                                Y_cam = (self.cy - v) * Z_cam / self.fy
                                
                                rob_x = Z_cam - self.cam_offset_x
                                rob_y = -X_cam
                                rob_z = self.cam_height + Y_cam
                                
                                # Güvenlik Sınırı
                                if (0.15 < rob_x < 0.60) and (0.0 < rob_z < 0.60):
                                    
                                    # Mesafe Kontrolü
                                    dist = math.sqrt(
                                        (rob_x - self.last_target_xyz[0])**2 + 
                                        (rob_y - self.last_target_xyz[1])**2 + 
                                        (rob_z - self.last_target_xyz[2])**2
                                    )

                                    if dist > self.movement_threshold:
                                        self.get_logger().info(f"YENI HEDEF -> X:{rob_x:.2f} Y:{rob_y:.2f} Z:{rob_z:.2f}")
                                        angles = self.solve_ik(rob_x, rob_y, rob_z)
                                        
                                        if angles:
                                            self.last_target_xyz = [rob_x, rob_y, rob_z]
                                            self.send_goal_to_moveit(angles)
                                            status_text = "PLANLANIYOR..."
                                    else:
                                        status_text = f"SABIT (<{self.movement_threshold*100:.1f}cm)"
                                        status_color = (255, 0, 0) # Mavi
                                else:
                                    status_text = "GUVENLI ALAN DISI"
                    except Exception:
                        pass

        # Bilgi Kutusu
        cv2.rectangle(frame, (0,0), (640, 40), (0,0,0), -1)
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.imshow("Robot Gozu", frame)
        cv2.waitKey(1)

    def send_goal_to_moveit(self, angles):
        self.robot_is_busy = True
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'my_arm'
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 1.0
        goal_msg.request.max_velocity_scaling_factor = 0.5 
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        joint_names = ['joint1', 'joint2', 'joint3']
        for i in range(3):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = angles[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)

        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.robot_is_busy = False
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.robot_is_busy = False

    def solve_ik(self, x, y, z):
        try:
            theta1 = math.atan2(y, x)
            r = math.sqrt(x**2 + y**2)
            z_offset = z - self.L1
            D = math.sqrt(r**2 + z_offset**2)
            max_reach = self.L2 + self.L3
            if D > max_reach: D = max_reach
            
            cos_elbow = (self.L2**2 + self.L3**2 - D**2) / (2 * self.L2 * self.L3)
            cos_elbow = max(min(cos_elbow, 1.0), -1.0)
            theta3 = math.pi - math.acos(cos_elbow)
            
            beta = math.atan2(z_offset, r)
            cos_alpha = (self.L2**2 + D**2 - self.L3**2) / (2 * self.L2 * D)
            cos_alpha = max(min(cos_alpha, 1.0), -1.0)
            theta2 = (math.pi / 2) - (beta + math.acos(cos_alpha))
            return [float(theta1), float(theta2), float(theta3)]
        except:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
