#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from ultralytics import YOLO

# ==========================================
# ÇALIŞMA ALANI TANIMI
# ==========================================
FRAME_W = 640
FRAME_H = 480
ROBOT_X_M = 0.40
ROBOT_Y_M = 0.30
ROBOT_Z_M = 0.20

def pixel_to_robot(cx, cy):
    X_robot = (cx / FRAME_W) * ROBOT_X_M
    Y_robot = (cy / FRAME_H) * ROBOT_Y_M
    Z_robot = ROBOT_Z_M
    return X_robot, Y_robot, Z_robot

# ==========================================
# HAFIZA SINIFI
# ==========================================
class ObjectMemory:
    def __init__(self, max_age=3.0):
        self.objects = {}
        self.max_age = max_age

    def update_frame(self, detections):
        now = time.time()
        new_objects = {}

        label_count = {}
        for label, cx, cy, rx, ry, rz in detections:
            count = label_count.get(label, 0)
            key = f"{label}_{count}"
            label_count[label] = count + 1

            new_objects[key] = {
                "label": label,
                "pixel": (cx, cy),
                "robot": (rx, ry, rz),
                "last_seen": now
            }

        self.objects = {
            k: v for k, v in self.objects.items()
            if now - v["last_seen"] < self.max_age
            and k not in new_objects
        }
        self.objects.update(new_objects)

    def get_all(self):
        now = time.time()
        self.objects = {
            k: v for k, v in self.objects.items()
            if now - v["last_seen"] < self.max_age
        }
        return self.objects


class YoloDetector(Node):
    def __init__(self):
        super().__init__('detector_yolo')

        self.publisher_ = self.create_publisher(Point, '/camera/target_coords', 10)
        self.model = YOLO("/home/orhan/ros2_ws/src/kinect_arm_control/scripts/best.pt")
        self.bridge = CvBridge()
        self.memory = ObjectMemory(max_age=3.0)

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )

        self.target_class = 'plastik'
        self.min_confidence = 0.60

        # ==========================================
        # PİKSEL YUMUŞATMA — USB kamera titremesi için
        # ==========================================
        self.smooth_cx = 0
        self.smooth_cy = 0
        self.alpha = 0.3  # düşürürsen daha yavaş ama daha sakin
                          # artırırsan daha hızlı ama daha titrek

        # ==========================================
        # STABİLİTE FİLTRESİ — USB kameraya göre ayarlı
        # ==========================================
        self.candidate_x = 0.0
        self.candidate_y = 0.0
        self.candidate_z = 0.0

        self.stable_count = 0
        self.required_stable_frames = 10    # 15'ten 10'a düşürüldü
        self.stability_threshold = 0.030    # 0.015'ten 0.030'a çıkarıldı
        self.publish_threshold = 0.025      # 0.01'den 0.025'e çıkarıldı

        self.last_published_x = 0.0
        self.last_published_y = 0.0
        self.last_published_z = 0.0

        self.get_logger().info("✅ USB Kamera Modu Aktif (Yumuşatma + Stabil Filtre)...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image, verbose=False)

            detections = []

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    conf = float(box.conf[0])

                    if conf < self.min_confidence:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # Ham merkez
                    raw_cx = (x1 + x2) // 2
                    raw_cy = (y1 + y2) // 2

                    # ==========================================
                    # PİKSEL YUMUŞATMA — titreme azaltma
                    # ==========================================
                    self.smooth_cx = int(
                        self.alpha * raw_cx + (1 - self.alpha) * self.smooth_cx
                    )
                    self.smooth_cy = int(
                        self.alpha * raw_cy + (1 - self.alpha) * self.smooth_cy
                    )
                    cx, cy = self.smooth_cx, self.smooth_cy

                    X_robot, Y_robot, Z_robot = pixel_to_robot(cx, cy)

                    detections.append((class_name, cx, cy, X_robot, Y_robot, Z_robot))

                    # ==========================================
                    # STABİLİTE KONTROLÜ — sadece hedef sınıf
                    # ==========================================
                    if class_name == self.target_class:
                        dist_to_candidate = math.sqrt(
                            (X_robot - self.candidate_x)**2 +
                            (Y_robot - self.candidate_y)**2
                            # Z sabit olduğu için dahil edilmedi
                        )

                        if dist_to_candidate < self.stability_threshold:
                            self.stable_count += 1
                            color = (0, 255, 255)
                            status_text = "BEKLIYOR..."

                            if self.stable_count >= self.required_stable_frames:
                                dist_to_published = math.sqrt(
                                    (X_robot - self.last_published_x)**2 +
                                    (Y_robot - self.last_published_y)**2
                                )

                                if dist_to_published > self.publish_threshold:
                                    point_msg = Point()
                                    point_msg.x = float(X_robot)
                                    point_msg.y = float(Y_robot)
                                    point_msg.z = float(Z_robot)
                                    self.publisher_.publish(point_msg)

                                    self.last_published_x = X_robot
                                    self.last_published_y = Y_robot
                                    self.last_published_z = Z_robot

                                    print(f"🚀 GONDERILDI -> X:{X_robot:.3f} | Y:{Y_robot:.3f} | Z:{Z_robot:.3f}")
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

                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                        cv2.circle(cv_image, (cx, cy), 5, color, -1)
                        cv2.putText(cv_image, f"{class_name} {conf:.2f}",
                                    (x1, max(15, y1 - 25)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                        cv2.putText(cv_image, status_text,
                                    (x1, max(15, y1 - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    else:
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(cv_image, f"{class_name} {conf:.2f}",
                                    (x1, max(15, y1 - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            # ==========================================
            # HAFIZA GÜNCELLE
            # ==========================================
            self.memory.update_frame(detections)

            # ==========================================
            # SOL ÜST KÖŞE PANELİ
            # ==========================================
            all_objects = self.memory.get_all()
            panel_h = 20 + len(all_objects) * 22
            cv2.rectangle(cv_image, (5, 5), (310, panel_h), (0, 0, 0), -1)
            cv2.putText(cv_image, "Son Gorulen Nesneler:", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            for i, (key, obj) in enumerate(all_objects.items()):
                rx, ry, rz = obj["robot"]
                text = f"  {obj['label']} -> {rx:.3f}m  {ry:.3f}m"
                cv2.putText(cv_image, text, (10, 42 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

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