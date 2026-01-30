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
        self.publisher_ = self.create_publisher(Point, '/camera/target_coords', 10)
        self.model = YOLO("yolov8n.pt") 
        self.bridge = CvBridge()
        
        # Kinect Topicleri
        self.image_sub = self.create_subscription(Image, '/kinect/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/kinect/depth/image_raw', self.depth_callback, 10)
        
        self.latest_depth_image = None
        self.target_class = 'cell phone' 
        
        # Kinect V1 Optik Sabitleri
        self.fx, self.fy = 554.25, 554.25
        self.cx, self.cy = 320.0, 240.0
        
        self.get_logger().info(f"🤖 YOLO + Kinect Hazır! Hedef: {self.target_class}")

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
            
            for result in results:
                for box in result.boxes:
                    if self.model.names[int(box.cls[0])] == self.target_class:
                        if float(box.conf[0]) < 0.6: continue

                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        
                        h, w = self.latest_depth_image.shape
                        if 0 <= cx < w and 0 <= cy < h:
                            depth_val = self.latest_depth_image[cy, cx]
                            if depth_val > 0:
                                # KAMERA EKSENLERİ
                                cam_z = depth_val / 1000.0  # İleri (Derinlik)
                                cam_x = (cx - self.cx) * cam_z / self.fx # Sağ/Sol
                                cam_y = (cy - self.cy) * cam_z / self.fy # Yukarı/Aşağı
                                
                                # ROBOT EKSENLERİNE EŞLEME (Mapping)
                                # Robot X (İleri) = Kamera Z (Derinlik) - 0.5m (Ofset)
                                # Robot Y (Sağ/Sol) = Kamera -X (Kamera ters duruyorsa X yapabilirsin)
                                # Robot Z (Yukarı/Aşağı) = Kamera -Y
                                
                                point_msg = Point()
                                point_msg.x = float(cam_z - 0.5) 
                                point_msg.y = float(-cam_x)      
                                point_msg.z = float(-cam_y + 0.15) # +0.15m yerden yükseltme
                                
                                # Güvenlik Sınırı
                                point_msg.x = max(0.1, min(point_msg.x, 0.6))
                                
                                self.publisher_.publish(point_msg)
                                
                                # Debug Görseli
                                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                cv2.putText(cv_image, f"X:{point_msg.x:.2f} Y:{point_msg.y:.2f}", (x1, y1-10), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                break 
            cv2.imshow("YOLO Debug", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"YOLO Hatası: {e}")

def main():
    rclpy.init()
    rclpy.spin(YoloDetector())
    rclpy.shutdown()