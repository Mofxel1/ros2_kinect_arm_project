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
        self.min_confidence = 0.40
        
        # Kamera Parametreleri
        self.fx, self.fy = 554.25, 554.25
        self.cx, self.cy = 320.0, 240.0

        # ==========================================
        # STABİLİTE (DUR-KALK) FİLTRESİ
        # ==========================================
        self.candidate_x = 0.0
        self.candidate_y = 0.0
        self.candidate_z = 0.0
        
        self.stable_count = 0
        self.required_stable_frames = 15  
        self.stability_threshold = 0.015  
        
        self.last_published_x = 0.0
        self.last_published_y = 0.0
        self.last_published_z = 0.0
        self.publish_threshold = 0.01     
        
        self.get_logger().info("✅ 42cm TCP Menzilli Stabil Mod Aktif (Max Uzanma: 40cm)...")

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
                                Z_cam = depth_val / 1000.0          
                                X_cam = (cx - self.cx) * Z_cam / self.fx 
                                Y_cam = (cy - self.cy) * Z_cam / self.fy 
                                
                                # ==========================================
                                # 2. UZAYSAL DÖNÜŞÜM
                                # ==========================================
                                X_robot = Z_cam - 0.50   
                                Y_robot = -X_cam         
                                Z_robot = -Y_cam + 0.30  

                                # ==========================================
                                # 3. YENİ 42 CM'LİK MENZİLE GÖRE SINIRLAR
                                # L2(22cm) + L3/TCP(20cm) = 42cm maksimum erişim!
                                # Güvenlik payı ile robotu en fazla 40cm ileri itiyoruz.
                                # ==========================================
                                X_robot = max(0.10, min(X_robot, 0.40))  
                                Z_robot = max(0.02, min(Z_robot, 0.45))  

                                # ==========================================
                                # 4. STABİLİTE (DURMA) KONTROLÜ
                                # ==========================================
                                dist_to_candidate = math.sqrt((X_robot - self.candidate_x)**2 + 
                                                              (Y_robot - self.candidate_y)**2 + 
                                                              (Z_robot - self.candidate_z)**2)
                                
                                if dist_to_candidate < self.stability_threshold:
                                    self.stable_count += 1
                                    color = (0, 255, 255) 
                                    status_text = "BEKLIYOR..."
                                    
                                    if self.stable_count >= self.required_stable_frames:
                                        dist_to_published = math.sqrt((X_robot - self.last_published_x)**2 + 
                                                                      (Y_robot - self.last_published_y)**2 + 
                                                                      (Z_robot - self.last_published_z)**2)
                                        
                                        if dist_to_published > self.publish_threshold:
                                            point_msg = Point()
                                            point_msg.x, point_msg.y, point_msg.z = float(X_robot), float(Y_robot), float(Z_robot)
                                            self.publisher_.publish(point_msg)
                                            
                                            self.last_published_x = X_robot
                                            self.last_published_y = Y_robot
                                            self.last_published_z = Z_robot
                                            
                                            print(f"🚀 GONDERILDI -> X:{X_robot:.2f} | Y:{Y_robot:.2f} | Z:{Z_robot:.2f}")
                                            color = (0, 255, 0) 
                                            status_text = "GONDERILDI!"
                                        else:
                                            color = (255, 0, 0) 
                                            status_text = "ALINMASI BEKLENIYOR"
                                else:
                                    self.candidate_x = X_robot
                                    self.candidate_y = Y_robot
                                    self.candidate_z = Z_robot
                                    self.stable_count = 0
                                    color = (0, 0, 255) 
                                    status_text = "HAREKET HALINDE"

                                # Ekrana Durum Çizimi
                                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                                label = f"X:{X_robot:.2f} Y:{Y_robot:.2f} Z:{Z_robot:.2f}" 
                                cv2.putText(cv_image, label, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                                cv2.putText(cv_image, status_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

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
