#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('detector_yolo')
        
        # --- BRAIN İLE BAĞLANTI NOKTASI ---
        # Senin Brain kodun '/camera/target_coords' dinliyor, biz de oraya yazıyoruz.
        self.publisher_ = self.create_publisher(Point, '/camera/target_coords', 10)
        
        # 1. YOLO Modelini Yükle
        # 'yolov8n.pt' en hızlısıdır. İlk çalışmada otomatik indirir.
        self.model = YOLO("yolov8n.pt") 
        
        self.bridge = CvBridge()
        
        # Kamera Abonelikleri
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        self.latest_depth_image = None
        
        # ARANACAK NESNE (COCO listesinden seçebilirsin: cup, bottle, cell phone...)
        self.target_class = 'sports ball' 
        
        # Kamera Parametreleri (Kinect V1 Tahmini)
        self.fx, self.fy = 554.25, 554.25
        self.cx, self.cy = 320.0, 240.0
        
        self.get_logger().info(f"🤖 YOLO Dedektörü Başlatıldı! Aranıyor: {self.target_class}")

    def depth_callback(self, msg):
        try:
            # Derinlik verisini ROS formatından OpenCV formatına çevir
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Derinlik hatası: {e}")

    def image_callback(self, msg):
        if self.latest_depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # --- YOLO İLE NESNE BUL ---
            results = self.model(cv_image, verbose=False)
            
            object_found = False

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Sınıf kontrolü (Sadece istediğimiz nesne mi?)
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    
                    if class_name == self.target_class:
                        # 1. Kutu Koordinatlarını Al
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        # 2. Merkez Noktayı Bul
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        
                        # 3. Derinlik Resminden Mesafeyi Oku
                        # (Görüntü sınırlarını taşmasın diye kontrol et)
                        h, w = self.latest_depth_image.shape
                        if 0 <= cx < w and 0 <= cy < h:
                            depth_val = self.latest_depth_image[cy, cx]
                            
                            # 0 değeri hatalı okumadır, onu atla
                            if depth_val > 0:
                                # 4. Pixel -> 3D Koordinat Dönüşümü (Matematik)
                                Z = depth_val / 1000.0  # mm -> metre
                                X = (cx - self.cx) * Z / self.fx
                                Y = (cy - self.cy) * Z / self.fy
                                
                                # 5. Brain'e Gönder (Topic: /camera/target_coords)
                                point_msg = Point()
                                point_msg.x = float(X)
                                point_msg.y = float(Y)
                                point_msg.z = float(Z)
                                self.publisher_.publish(point_msg)
                                
                                object_found = True
                                
                                # 6. Görselleştirme (Ekrana Çiz)
                                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                                cv2.putText(cv_image, f"{class_name} {Z:.2f}m", (x1, y1-10), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("YOLO Brain Input", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"YOLO Hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
