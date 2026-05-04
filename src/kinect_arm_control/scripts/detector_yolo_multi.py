#!/usr/bin/env python3

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO


class TrackedObject:
    def __init__(self, x, y, z, confidence, class_id, class_name):
        self.x = x
        self.y = y
        self.z = z
        self.confidence = confidence
        self.class_id = class_id
        self.class_name = class_name
        self.stable_count = 1
        self.last_seen_time = time.time()

    def distance_to(self, x, y, z):
        return math.sqrt(
            (self.x - x) ** 2 +
            (self.y - y) ** 2 +
            (self.z - z) ** 2
        )

    def update(self, x, y, z, confidence):
        # Basit yumuşatma
        alpha = 0.65
        self.x = alpha * self.x + (1.0 - alpha) * x
        self.y = alpha * self.y + (1.0 - alpha) * y
        self.z = alpha * self.z + (1.0 - alpha) * z
        self.confidence = max(self.confidence, confidence)
        self.stable_count += 1
        self.last_seen_time = time.time()


class YoloMultiDetector(Node):
    def __init__(self):
        super().__init__("detector_yolo_multi")

        self.publisher_ = self.create_publisher(Float32MultiArray, "/detected_objects", 10)

        # Eski çalışan kodundaki model yolu korunuyor
        self.model = YOLO("/home/orhan/ros2_ws/src/kinect_arm_control/scripts/best.pt")
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/depth/image_raw", self.depth_callback, 10)

        self.latest_depth_image = None

        # Şimdilik sadece plastik. İleride buraya metal/cam eklersin.
        self.allowed_classes = ["plastik"]

        self.min_confidence = 0.40

        # Kamera parametreleri eski çalışan koddakiyle aynı
        self.fx, self.fy = 554.25, 554.25
        self.cx, self.cy = 320.0, 240.0

        # Takip/stabilite ayarları
        self.tracks = []
        self.track_match_distance = 0.035       # 3.5 cm yakınsa aynı obje say
        self.required_stable_frames = 5         # obje kaç frame stabil görünmeli
        self.track_timeout_sec = 1.0            # 1 sn görünmeyen track silinir

        # Publish frekansı sınırlama
        self.last_publish_time = 0.0
        self.publish_period_sec = 0.20          # 5 Hz

        self.get_logger().info("✅ YOLO MULTI aktif: /detected_objects yayınlanacak.")

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Derinlik hatası: {e}")

    def get_depth_median(self, cx, cy, w_depth, h_depth):
        y_min = max(0, cy - 5)
        y_max = min(h_depth, cy + 5)
        x_min = max(0, cx - 5)
        x_max = min(w_depth, cx + 5)

        roi = self.latest_depth_image[y_min:y_max, x_min:x_max]
        valid_depths = roi[(roi > 0) & (~np.isnan(roi))]

        if len(valid_depths) == 0:
            return None

        return float(np.median(valid_depths))

    def camera_to_robot(self, pixel_x, pixel_y, depth_mm):
        # 1. Kamera saf verisi
        z_cam = depth_mm / 1000.0
        x_cam = (pixel_x - self.cx) * z_cam / self.fx
        y_cam = (pixel_y - self.cy) * z_cam / self.fy

        # 2. Eski çalışan koddaki uzaysal dönüşüm korunuyor
        x_robot = z_cam - 0.50
        y_robot = -x_cam
        z_robot = -y_cam + 0.30

        # Önemli: Burada artık kırpma yok.
        # Ulaşılabilirlik task_manager içinde is_reachable() ile yapılacak.
        return x_robot, y_robot, z_robot

    def update_tracks(self, detections):
        now = time.time()

        # Eski trackleri temizle
        self.tracks = [
            t for t in self.tracks
            if now - t.last_seen_time <= self.track_timeout_sec
        ]

        for det in detections:
            x, y, z, conf, class_id, class_name = det

            best_track = None
            best_dist = 999.0

            for track in self.tracks:
                if track.class_id != class_id:
                    continue

                dist = track.distance_to(x, y, z)
                if dist < best_dist:
                    best_dist = dist
                    best_track = track

            if best_track is not None and best_dist < self.track_match_distance:
                best_track.update(x, y, z, conf)
            else:
                self.tracks.append(
                    TrackedObject(x, y, z, conf, class_id, class_name)
                )

    def publish_stable_tracks(self):
        now = time.time()
        if now - self.last_publish_time < self.publish_period_sec:
            return

        self.last_publish_time = now

        msg = Float32MultiArray()
        data = []

        stable_tracks = [
            t for t in self.tracks
            if t.stable_count >= self.required_stable_frames
        ]

        for t in stable_tracks:
            data.extend([
                float(t.x),
                float(t.y),
                float(t.z),
                float(t.confidence),
                float(t.class_id),
            ])

        msg.data = data
        self.publisher_.publish(msg)

        if stable_tracks:
            text = " | ".join([
                f"{t.class_name}: x={t.x:.2f}, y={t.y:.2f}, z={t.z:.2f}, conf={t.confidence:.2f}"
                for t in stable_tracks
            ])
            self.get_logger().info(f"📦 Stabil objeler: {text}")

    def image_callback(self, msg):
        if self.latest_depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image, verbose=False)

            h_depth, w_depth = self.latest_depth_image.shape
            detections = []

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    conf = float(box.conf[0])

                    if class_name not in self.allowed_classes:
                        continue

                    if conf < self.min_confidence:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    px = (x1 + x2) // 2
                    py = (y1 + y2) // 2

                    if not (0 <= px < w_depth and 0 <= py < h_depth):
                        continue

                    depth_val = self.get_depth_median(px, py, w_depth, h_depth)
                    if depth_val is None:
                        continue

                    x_robot, y_robot, z_robot = self.camera_to_robot(px, py, depth_val)

                    detections.append((
                        x_robot,
                        y_robot,
                        z_robot,
                        conf,
                        cls_id,
                        class_name
                    ))

                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    label = f"{class_name} {conf:.2f} | X:{x_robot:.2f} Y:{y_robot:.2f} Z:{z_robot:.2f}"
                    cv2.putText(
                        cv_image,
                        label,
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        2
                    )

            self.update_tracks(detections)
            self.publish_stable_tracks()

            cv2.imshow("YOLO Multi Detector", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"YOLO multi hata: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloMultiDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()