#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('detector_color')
        
        # --- BRAIN İLE BAĞLANTI NOKTASI ---
        self.publisher_ = self.create_publisher(Point, '/camera/target_coords', 10)
        
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        self.latest_depth_image = None
        
        # YEŞİL RENK ARALIĞI (HSV)
        self.lower_green = np.array([35, 100, 100])
        self.upper_green = np.array([85, 255, 255])
        
        # Kamera Parametreleri
        self.fx, self.fy = 554.25, 554.25
        self.cx, self.cy = 320.0, 240.0
        
        self.get_logger().info("🎨 Renk Dedektörü Başlatıldı (Yeşil Hedef)")

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Derinlik hatası: {e}")

    def image_callback(self, msg):
        if self.latest_depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Maskeleme
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # En büyük nesneyi bul
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                
                if radius > 10: # Çok küçükleri görmezden gel
                    cx, cy = int(x), int(y)
                    
                    # Derinlik oku
                    h, w = self.latest_depth_image.shape
                    if 0 <= cx < w and 0 <= cy < h:
                        depth_val = self.latest_depth_image[cy, cx]
                        
                        if depth_val > 0:
                            Z = depth_val / 1000.0
                            X = (cx - self.cx) * Z / self.fx
                            Y = (cy - self.cy) * Z / self.fy
                            
                            # Brain'e Gönder
                            point_msg = Point()
                            point_msg.x = float(X)
                            point_msg.y = float(Y)
                            point_msg.z = float(Z)
                            self.publisher_.publish(point_msg)
                            
                            # Çizim
                            cv2.circle(cv_image, (cx, cy), int(radius), (0, 255, 255), 2)
                            cv2.putText(cv_image, f"X:{X:.2f} Z:{Z:.2f}", (cx-20, cy-20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv2.imshow("Color Detector Input", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Görüntü hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
