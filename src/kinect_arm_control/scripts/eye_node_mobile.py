#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np
from ultralytics import YOLO
import math

class MobileEyeNode(Node):
    def __init__(self):
        super().__init__('eye_node_mobile')
        self.target_object = "bottle"
        self.min_confidence = 0.50  # %65 altındaki tahminleri yok sayar
        self.coord_pub = self.create_publisher(Point, '/camera/target_coords', 10)
        self.model = YOLO("yolov8n.pt")
        
        # Kamera ayarları (0: Dahili, 1: DroidCam/Harici)
        self.camera_index = 0 
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().error("❌ Kamera bulunamadı! İndeksi kontrol et.")
        
        self.last_sent_coords = [0.0, 0.0, 0.0]
        self.min_dist_change = 0.02 # 2 cm'den az hareketleri gönderme (titremeyi engeller)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"📱 Mobil Göz Hazır! Hedef: {self.target_object} (Conf: {self.min_confidence})")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        # YOLO ile tespit yap
        results = self.model(frame, verbose=False)
        
        found = False
        for r in results:
            for box in r.boxes:
                # GÜVEN KONTROLÜ (Yanlış algılamayı önleyen kısım)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                if label == self.target_object and conf >= self.min_confidence:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # Derinlik tahmini (Kutu yüksekliği üzerinden)
                    box_height = y2 - y1
                    estimated_z = 80.0 / box_height if box_height > 0 else 0.5
                    estimated_z = max(0.2, min(estimated_z, 0.6)) 

                    # Normalizasyon (-1.0 ile 1.0 arası)
                    norm_x = (cx - 320) / 320.0
                    norm_y = (cy - 240) / 240.0

                    # Robot koordinatları
                    target_x = float(estimated_z)
                    target_y = float(-norm_x * 0.4)
                    target_z = float(-norm_y * 0.3 + 0.2)

                    # Hareket Filtresi (Sadece nesne yer değiştirdiğinde mesaj at)
                    dist = math.sqrt((target_x - self.last_sent_coords[0])**2 + 
                                     (target_y - self.last_sent_coords[1])**2 + 
                                     (target_z - self.last_sent_coords[2])**2)

                    if dist > self.min_dist_change:
                        msg = Point()
                        msg.x, msg.y, msg.z = target_x, target_y, target_z
                        self.coord_pub.publish(msg)
                        self.last_sent_coords = [target_x, target_y, target_z]
                        self.get_logger().info(f"🎯 Telefon Tespit Edildi (%{conf*100:.0f}): X:{target_x:.2f}")

                    # Görselleştirme
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} %{conf*100:.0f}", (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    found = True
                    break 
            if found: break

        cv2.imshow("Mobile YOLO View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main():
    rclpy.init()
    node = MobileEyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()