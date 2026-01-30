#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('detector_yolo')
        
        self.publisher_ = self.create_publisher(Point, '/camera/target_coords', 10)
        self.model = YOLO("yolov8n.pt") 
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, '/kinect/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/kinect/depth/image_raw', self.depth_callback, 10)
        
        self.latest_depth_image = None
        self.target_class = 'cell phone' 
        self.min_confidence = 0.60
        
        # Kamera Parametreleri
        self.fx, self.fy = 554.25, 554.25
        self.cx, self.cy = 320.0, 240.0

        # Titreme Filtresi
        self.last_sent_x = 0.0
        self.last_sent_y = 0.0
        self.last_sent_z = 0.0
        self.movement_threshold = 0.02 
        
        self.get_logger().info("Eksen Kalibrasyonu Modu Aktif...")

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Derinlik hatası: {e}")

    def image_callback(self, msg):
        if self.latest_depth_image is None: return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image, verbose=False)
            h_depth, w_depth = self.latest_depth_image.shape
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    conf = float(box.conf[0])
                    
                    if class_name == self.target_class and conf > self.min_confidence:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        
                        if 0 <= cx < w_depth and 0 <= cy < h_depth:
                            depth_val = self.latest_depth_image[cy, cx]
                            
                            if depth_val > 0 and not np.isnan(depth_val):
                                # 1. KAMERA SAF VERİSİ
                                Z_cam = depth_val / 1000.0          # Derinlik (Kameradan uzaklaşma)
                                X_cam = (cx - self.cx) * Z_cam / self.fx # Kamera Sağı (+) / Solu (-)
                                Y_cam = (cy - self.cy) * Z_cam / self.fy # Kamera Aşağısı (+) / Yukarısı (-)
                                
                                # ==========================================
                                #       BURAYI DENEYEREK BULACAĞIZ
                                # ==========================================
                                
                                # SENARYO 1: Kamera Robotun ARKASINDA duruyor (-X yönünde)
                                # İleri itince Robot X artmalı
                                # Sağa çekince Robot Y azalmalı (veya artmalı)
                                # Yukarı çekince Robot Z artmalı
                                
                                # Mevcut ayar (Bunu dene):
                                X_robot = Z_cam - 0.50   # Derinlik -> Robot İleri (X)
                                Y_robot = -X_cam         # Kamera Sağı -> Robot Sağı (-Y)
                                Z_robot = -Y_cam + 0.20  # Kamera Yukarısı -> Robot Yukarısı (Z)

                                # ------------------------------------------
                                # SENARYO 2: (Eğer robotun sağı solu ters ise bunu aç)
                                # Y_robot = X_cam 
                                # ------------------------------------------
                                
                                # ------------------------------------------
                                # SENARYO 3: (Eğer İleri itince robot yukarı kalkıyorsa bunu aç)
                                # X_robot = -Y_cam + 0.30  
                                # Z_robot = Z_cam - 0.20
                                # ------------------------------------------

                                # Güvenlik Sınırları (Simülasyonda uçup gitmesin)
                                X_robot = max(0.10, min(X_robot, 0.60)) # Min 10cm, Max 60cm ileri
                                Z_robot = max(-0.20, min(Z_robot, 0.50)) # Yerden çok aşağı veya yukarı gitmesin

                                # Filtre ve Gönderim
                                dist = math.sqrt((X_robot - self.last_sent_x)**2 + (Y_robot - self.last_sent_y)**2 + (Z_robot - self.last_sent_z)**2)

                                if dist > self.movement_threshold:
                                    point_msg = Point()
                                    point_msg.x, point_msg.y, point_msg.z = float(X_robot), float(Y_robot), float(Z_robot)
                                    self.publisher_.publish(point_msg)
                                    
                                    self.last_sent_x = X_robot
                                    self.last_sent_y = Y_robot
                                    self.last_sent_z = Z_robot
                                    
                                    # HANGİ EKSENİN HANGİSİ OLDUĞUNU GÖRMEK İÇİN LOG
                                    print(f"🎯 GONDERILEN: İleri(X):{X_robot:.2f} | Yan(Y):{Y_robot:.2f} | Yükseklik(Z):{Z_robot:.2f}")
                                    color = (0, 255, 0)
                                else:
                                    color = (0, 255, 255)

                                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                                label = f"X:{X_robot:.2f} Y:{Y_robot:.2f} Z:{Z_robot:.2f}" 
                                cv2.putText(cv_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            cv2.imshow("YOLO Brain Input", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Hata: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
