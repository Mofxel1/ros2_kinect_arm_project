#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import serial
import math

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        
        # MoveIt'in onayladığı yörüngeleri dinliyoruz
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            10)

        # Seri Port Bağlantısı (STM32 USB CDC genelde ttyACM0 veya ttyUSB0 olur)
        self.serial_port = '/dev/ttyACM0'
        self.baudrate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"✅ STM32'ye Basariyla Baglanildi: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"❌ Seri Port Hatasi! STM32 takili mi? Hata: {e}")
            self.ser = None

        # Robotun Gerçekteki Mevcut Açılarını Hafızada Tutuyoruz
        # URDF'deki joint1 = Omuz (M1), joint2 = Dirsek (M2) olarak varsayılmıştır.
        self.current_rad = {'joint1': 0.0, 'joint2': 0.0}

    def trajectory_callback(self, msg):
        if not self.ser:
            self.get_logger().warn("Seri port kapalı, komut gönderilemedi.")
            return

        if len(msg.trajectory) == 0:
            return

        # Yörüngenin en sonundaki (hedef) noktayı alıyoruz
        last_point = msg.trajectory[0].joint_trajectory.points[-1]
        joint_names = msg.trajectory[0].joint_trajectory.joint_names

        try:
            idx_j1 = joint_names.index('joint1') # M1 (NEMA23 - Omuz)
            idx_j2 = joint_names.index('joint2') # M2 (NEMA17 - Dirsek)
        except ValueError:
            self.get_logger().warn("Yörüngede joint1 veya joint2 bulunamadı!")
            return

        target_j1_rad = last_point.positions[idx_j1]
        target_j2_rad = last_point.positions[idx_j2]

        # 1. DELTA (FARK) HESAPLAMA
        delta_j1 = target_j1_rad - self.current_rad['joint1']
        delta_j2 = target_j2_rad - self.current_rad['joint2']

        # 2. RADYAN -> STEP DÖNÜŞÜMÜ (3200 step = 2*Pi Radyan)
        steps_per_rad = 3200.0 / (2.0 * math.pi)
        steps_j1 = int(abs(delta_j1) * steps_per_rad)
        steps_j2 = int(abs(delta_j2) * steps_per_rad)

        # Gereksiz titremeleri (1-2 step) engellemek için filtre
        if steps_j1 < 5 and steps_j2 < 5:
            return 

        # 3. YÖN BELİRLEME (Fiziksel robot ters dönerse buradaki 0 ve 1'leri yer değiştir)
        dir_j1 = 0 if delta_j1 > 0 else 1
        dir_j2 = 0 if delta_j2 > 0 else 1

        # 4. HIZ (RPM) BELİRLEME
        rpm = 60 # STM32'deki güvenli sınırlarına göre sabit 60 RPM

        # 5. STM32 KOMUT DİZİSİNİ OLUŞTURMA (Örn: M1:800:0:60,M2:400:1:60\n)
        cmd_list = []
        if steps_j1 > 0:
            cmd_list.append(f"M1:{steps_j1}:{dir_j1}:{rpm}")
        if steps_j2 > 0:
            cmd_list.append(f"M2:{steps_j2}:{dir_j2}:{rpm}")

        if cmd_list:
            final_cmd = ",".join(cmd_list) + "\n"
            
            # STM32'ye Gönder
            self.ser.write(final_cmd.encode('utf-8'))
            self.get_logger().info(f"🚀 STM32'ye Ateslendi -> {final_cmd.strip()}")

            # Gerçek konumu güncelle (Böylece bir sonraki hareket kaldığı yerden hesaplanır)
            self.current_rad['joint1'] = target_j1_rad
            self.current_rad['joint2'] = target_j2_rad

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
