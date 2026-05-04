#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class ReachabilityMapNode(Node):
    def __init__(self):
        super().__init__('reachability_map_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/workspace_cloud', 1)
        self.timer = self.create_timer(1.0, self.publish_cloud)
        self.get_logger().info("3D Workspace (Nokta Bulutu) RViz'e gonderiliyor...")

        # Robot Anatomi Parametreleriniz
        self.L1, self.L2, self.L3 = 0.183, 0.220, 0.200
        self.OMUZ_OFFSET = 0.345
        self.DIRSEK_SARKMA = 1.510

    def publish_cloud(self):
        marker_array = MarkerArray()
        
        marker = Marker()
        # Referans Noktasını Base Link yapıyoruz ki doğru yere çizsin
        marker.header.frame_id = "base_link" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "reachability"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0 # Döndürme hatasını önlemek için sabitliyoruz
        
        # Nokta boyutu (2 cm)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # Şeffaf Mavi/Yeşil (Tatlı bir workspace rengi)
        marker.color.r = 0.0
        marker.color.g = 0.6
        marker.color.b = 0.8
        marker.color.a = 0.3 

        # AÇI TARAMASI VE KİNEMATİK
        # URDF Limitleriniz
        for t1 in self.frange(-3.14, 3.14, 0.1):      # J0 (Gövde): 360 derece tarama
            for t2 in self.frange(0.0, 0.80, 0.05):   # J1 (Omuz): Sadece limitleri dahilinde
                for t3 in self.frange(-1.0, 0.1, 0.05): # J2 (Dirsek): Sadece limitleri dahilinde
                    
                    # 1. Gerçek Açılar (Kalibrasyonlu)
                    theta_L2 = t2 + self.OMUZ_OFFSET
                    gamma = self.DIRSEK_SARKMA - t3
                    theta_L3 = theta_L2 - (math.pi - gamma)

                    # 2. X-Z Düzleminde (Profilden) Uzanma (R) ve Yükseklik (Z)
                    r = self.L2 * math.cos(theta_L2) + self.L3 * math.cos(theta_L3)
                    z = self.L1 + self.L2 * math.sin(theta_L2) + self.L3 * math.sin(theta_L3)

                    # 3. J0 ile R'yi X-Y düzlemine yayma (Kusursuz Kubbe)
                    x = r * math.cos(t1)
                    y = r * math.sin(t1)

                    # Noktayı Listeye Ekle
                    p = Point()
                    p.x, p.y, p.z = float(x), float(y), float(z)
                    marker.points.append(p)

        marker_array.markers.append(marker)
        self.publisher_.publish(marker_array)

    def frange(self, start, stop, step):
        i = start
        while i < stop:
            yield i
            i += step

def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
