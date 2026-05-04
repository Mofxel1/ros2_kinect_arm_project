#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.publisher_ = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.timer = self.create_timer(1.0, self.publish_obstacles)
        
        # --- AYARLAR ---
        self.num_baskets = 3       # Üretilecek sepet sayısı (İstediğin gibi değiştir)
        self.gap_between = 0.05    # Sepetler arası boşluk (Mesafe: 5 cm = 0.05 m)

    def create_box(self, dimensions, position):
        """Yardımcı Fonksiyon: Kutu (SolidPrimitive) ve Konum (Pose) oluşturur."""
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0 

        return primitive, pose

    def create_basket(self, basket_id, center_x, center_y, center_z):
        """Tek bir sepetin 5 duvarını oluşturup CollisionObject olarak döndürür."""
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = basket_id

        # SEPET BOYUTLARI
        t = 0.008
        w = 0.14
        d = 0.14
        h = 0.10

        # 1. TABAN (Zemin)
        prim, pose = self.create_box([d, w, t], [center_x, center_y, center_z])
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)

        # 2. ÖN DUVAR (Robot tarafı)
        prim, pose = self.create_box([t, w, h], [center_x - (d/2) + (t/2), center_y, center_z + (h/2)])
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)

        # 3. ARKA DUVAR
        prim, pose = self.create_box([t, w, h], [center_x + (d/2) - (t/2), center_y, center_z + (h/2)])
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)

        # 4. SOL DUVAR
        prim, pose = self.create_box([d - 2*t, t, h], [center_x, center_y + (w/2) - (t/2), center_z + (h/2)])
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)

        # 5. SAĞ DUVAR
        prim, pose = self.create_box([d - 2*t, t, h], [center_x, center_y - (w/2) + (t/2), center_z + (h/2)])
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)

        obj.operation = CollisionObject.ADD
        return obj

    def publish_obstacles(self):
        cx = -0.30  # Tüm sepetlerin robota uzaklığı (X ekseninde 30 cm ileride)
        cz = 0.00  # Tüm sepetlerin yerden yüksekliği (Z ekseninde 5 cm)
        
        w = 0.20   # Sepet genişliği (Yukarıdaki hesapla aynı olmalı)
        
        # Bir sepetin merkezinden diğerinin merkezine olan toplam mesafe (Genişlik + Boşluk)
        step = w + self.gap_between 

        # Sepet grubunu tam ortaya (Y=0) simetrik hizalamak için başlangıç noktasını hesapla
        start_y = ((self.num_baskets - 1) * step) / 2.0

        for i in range(self.num_baskets):
            # Her sepete benzersiz bir ID vermeliyiz (Örn: sepet_0, sepet_1, sepet_2)
            basket_id = f"ayristirma_sepeti_{i}"
            
            # Bu sepetin Y eksenindeki konumunu hesapla (Soldan sağa doğru diziyoruz)
            cy = start_y - (i * step) 

            # Sepeti oluştur ve yayınla
            basket_obj = self.create_basket(basket_id, cx, cy, cz)
            self.publisher_.publish(basket_obj)

        self.get_logger().info(f'🛒 {self.num_baskets} ADET SEPET RVIZ SAHNESINE EKLENDI! Çarpışma Denetimi Aktif.')
        self.timer.cancel() # Sadece bir kere gönder ve dur

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()