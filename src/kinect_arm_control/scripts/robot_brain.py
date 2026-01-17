#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from collections import deque
import statistics
import math

# MoveIt Kütüphaneleri (ROS 2 Python)
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class RobotBrainNode(Node):
    def __init__(self):
        super().__init__('robot_brain_node')

        # ---------------------------------------------------------
        # ⚙️ AYARLAR
        # ---------------------------------------------------------
        self.raw_topic_name = '/camera/target_coords'
        self.window_size = 15      # Median filtre için son 15 veri
        self.movement_threshold = 0.015 # 1.5 cm altındaki değişimleri yok say
        self.planning_group_name = "arm" # MoveIt Config'deki grup adın (Genelde 'arm' veya 'manipulator')
        # ---------------------------------------------------------

        # 1. Subscriber: Kameradan gelen koordinatı dinle
        self.subscription = self.create_subscription(
            Point, 
            self.raw_topic_name, 
            self.target_callback, 
            10
        )
        
        # 2. Tampon Bellekler (Buffer)
        self.buffer_x = deque(maxlen=self.window_size)
        self.buffer_y = deque(maxlen=self.window_size)
        self.buffer_z = deque(maxlen=self.window_size)

        # 3. Son Gidilen Konum (Hafıza)
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0

        # 4. MoveIt Başlatma (Robot Kolunu Hazırla) 🦾
        try:
            # MoveItPy nesnesini başlat (ROS 2 Humble için standart yöntem)
            self.robot = MoveItPy(node_name="moveit_py")
            self.arm_group = self.robot.get_planning_component(self.planning_group_name)
            self.get_logger().info(f'🦾 MoveIt Grubu "{self.planning_group_name}" Bağlandı!')
        except Exception as e:
            self.get_logger().error(f'❌ MoveIt Bağlantı Hatası: {e}')
            self.get_logger().warn('Robot hareket etmeyecek, sadece hesaplama yapacak.')
            self.arm_group = None

        self.get_logger().info('🧠 Robot Beyni Hazır! Veri bekleniyor...')

    def target_callback(self, msg):
        """Kameradan veri geldiğinde çalışır"""
        
        # Veriyi tampona at
        self.buffer_x.append(msg.x)
        self.buffer_y.append(msg.y)
        self.buffer_z.append(msg.z)

        # Yeterli veri yoksa bekle
        if len(self.buffer_x) < self.window_size:
            if len(self.buffer_x) % 5 == 0: # Log kirliliği olmasın diye ara sıra yaz
                self.get_logger().info(f'⏳ Veri toplanıyor... ({len(self.buffer_x)}/{self.window_size})')
            return

        # --- FİLTRELEME (MEDIAN) ---
        median_x = statistics.median(self.buffer_x)
        median_y = statistics.median(self.buffer_y)
        median_z = statistics.median(self.buffer_z)

        # --- DEADBAND KONTROLÜ (Ölü Bölge) ---
        diff_x = abs(median_x - self.current_target_x)
        diff_y = abs(median_y - self.current_target_y)
        diff_z = abs(median_z - self.current_target_z)
        
        total_change = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)

        if total_change > self.movement_threshold:
            # Eşik aşıldı -> HAREKET VAKTİ! 🚀
            self.get_logger().info(f'✅ Yeni Hedef Onaylandı: X:{median_x:.3f} Y:{median_y:.3f} Z:{median_z:.3f} (Fark: {total_change:.4f}m)')
            
            # Hafızayı güncelle
            self.current_target_x = median_x
            self.current_target_y = median_y
            self.current_target_z = median_z
            
            # Robotu hareket ettir
            self.move_robot_to_target(median_x, median_y, median_z)
        else:
            # Değişim çok küçük -> Göz ardı et
            pass

    def move_robot_to_target(self, x, y, z):
        """MoveIt kullanarak robotu hareket ettiren fonksiyon"""
        
        if self.arm_group is None:
            self.get_logger().warn("⚠️ MoveIt bağlı değil, hareket edilemiyor.")
            return

        try:
            # 1. Hedef Pozisyonu Oluştur
            # NOT: Robotun uç noktasının yönelimi (Orientation) sabit kalmalı.
            # Genelde aşağı bakması istenir. Aşağıdaki Quaternion değerleri "Aşağı Bak" demektir.
            # Eğer robotun sapıtıyorsa bu değerleri kendi robotuna göre düzenle.
            pose_goal = Pose()
            pose_goal.position.x = x
            pose_goal.position.y = y
            pose_goal.position.z = z
            
            # Yönelim (Quaternion) - Aşağı bakması için örnek değerler:
            pose_goal.orientation.w = 1.0
            pose_goal.orientation.x = 0.0
            pose_goal.orientation.y = 0.0
            pose_goal.orientation.z = 0.0

            # 2. Hedefi MoveIt'e Ver
            self.arm_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

            # 3. Planla ve Git
            self.get_logger().info("📐 Rota Planlanıyor...")
            plan_result = self.arm_group.plan()
            
            if plan_result:
                self.get_logger().info("🚀 Rota Bulundu! Hareket Başlıyor...")
                self.arm_group.execute()
                self.get_logger().info("🏁 Hedefe Varıldı!")
            else:
                self.get_logger().error("❌ Planlama Başarısız! Hedef erişilemez olabilir.")

        except Exception as e:
            self.get_logger().error(f"❌ Hareket Hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrainNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
