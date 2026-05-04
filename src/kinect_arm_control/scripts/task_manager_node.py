#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from reachability import ArmReachability


@dataclass
class DetectedObject:
    x: float
    y: float
    z: float
    confidence: float
    class_id: int
    class_name: str


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_node")

        self.detected_sub = self.create_subscription(
            Float32MultiArray,
            "/detected_objects",
            self.detected_objects_callback,
            10
        )

        self.done_sub = self.create_subscription(
            Bool,
            "/arm_motion_done",
            self.motion_done_callback,
            10
        )

        self.target_pub = self.create_publisher(Point, "/camera/target_coords", 10)
        self.status_pub = self.create_publisher(String, "/task_status", 10)

        self.reachability = ArmReachability()

        # YOLO class id -> isim eşlemesi.
        # Şimdilik task manager için güvenli fallback.
        # Detector zaten class_name'i msg içinde taşıyamadığı için class_id ile mapliyoruz.
        # Eğer modelde plastik id farklıysa logdan bakıp burayı düzelt.
        self.class_id_to_name = {
            0: "plastik",
            1: "metal",
            2: "cam",
            3: "yesil_cam",
            4: "kahverengi_cam",
            5: "seffaf_cam",
        }

        # Sınıfa göre bırakma/gösterme koordinatları.
        # Bunları gerçek sepet konumuna göre sonra kalibre ederiz.
        # Şimdilik tüm sınıflar aynı bırakma noktasına gitsin.
        # Bu nokta add_obstacle.py içindeki ortadaki sepetin 5 cm üstüdür.
        # Sepet:
        #   center_x = -0.30
        #   center_y = 0.00
        #   center_z = 0.00
        #   height   = 0.15
        # Bırakma noktası:
        #   z = center_z + height + 0.05 = 0.20
        basket_drop_target = Point(x=-0.30, y=0.00, z=0.20)

        self.class_targets = {
            "plastik": basket_drop_target,
            "metal": basket_drop_target,
            "cam": basket_drop_target,
            "yesil_cam": basket_drop_target,
            "kahverengi_cam": basket_drop_target,
            "seffaf_cam": basket_drop_target,
        }
        

        self.home_target = Point(x=0.30, y=0.18, z=0.20)

        self.detected_objects = []
        self.done_targets = []

        self.current_object = None
        self.current_target_type = None

        self.state = "WAIT_OBJECTS"

        self.last_target_sent_time = 0.0
        self.motion_timeout_sec = 12.0

        # Aynı objeyi tekrar işlememek için mesafe eşiği
        self.done_distance_threshold = 0.04

        # Çok sık hedef basmayı engelle
        self.min_seconds_between_targets = 1.0

        self.timer = self.create_timer(0.3, self.state_loop)

        self.check_class_targets()

        self.get_logger().info("✅ Task Manager aktif: obje → sınıf hedefi → home döngüsü hazır.")

    def check_class_targets(self):
        for class_name, point in self.class_targets.items():
            result = self.reachability.is_reachable(point.x, point.y, point.z)
            if not result.reachable:
                self.get_logger().warn(
                    f"⚠️ {class_name} hedefi ulaşılmaz görünüyor: "
                    f"x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}, sebep={result.reason}"
                )
            else:
                self.get_logger().info(
                    f"✅ {class_name} hedefi ulaşılabilir: "
                    f"j0={result.joint0:.2f}, j1={result.joint1:.2f}, j2={result.joint2:.2f}"
                )

        home_result = self.reachability.is_reachable(
            self.home_target.x,
            self.home_target.y,
            self.home_target.z
        )

        if not home_result.reachable:
            self.get_logger().warn(
                f"⚠️ HOME hedefi ulaşılmaz görünüyor: sebep={home_result.reason}"
            )
        else:
            self.get_logger().info("✅ HOME hedefi ulaşılabilir.")

    def detected_objects_callback(self, msg):
        objects = []

        if len(msg.data) % 5 != 0:
            self.get_logger().warn(
                f"/detected_objects formatı hatalı. Veri uzunluğu: {len(msg.data)}"
            )
            return

        for i in range(0, len(msg.data), 5):
            x = float(msg.data[i])
            y = float(msg.data[i + 1])
            z = float(msg.data[i + 2])
            conf = float(msg.data[i + 3])
            class_id = int(msg.data[i + 4])

            class_name = self.class_id_to_name.get(class_id, f"class_{class_id}")

            objects.append(
                DetectedObject(
                    x=x,
                    y=y,
                    z=z,
                    confidence=conf,
                    class_id=class_id,
                    class_name=class_name
                )
            )

        self.detected_objects = objects

    def motion_done_callback(self, msg):
        if msg.data:
            self.get_logger().info(f"✅ Hareket tamamlandı: {self.current_target_type}")
            self.handle_motion_success()
        else:
            self.get_logger().warn(f"⚠️ Hareket başarısız: {self.current_target_type}")
            self.handle_motion_failed()

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def distance(self, a, b):
        return math.sqrt(
            (a.x - b.x) ** 2 +
            (a.y - b.y) ** 2 +
            (a.z - b.z) ** 2
        )

    def object_to_point(self, obj):
        p = Point()
        p.x = float(obj.x)
        p.y = float(obj.y)
        p.z = float(obj.z)
        return p

    def is_done_before(self, obj):
        obj_point = self.object_to_point(obj)

        for done_point in self.done_targets:
            if self.distance(obj_point, done_point) < self.done_distance_threshold:
                return True

        return False

    def filter_reachable_objects(self):
        valid = []

        for obj in self.detected_objects:
            if self.is_done_before(obj):
                continue

            result = self.reachability.is_reachable(obj.x, obj.y, obj.z)

            if not result.reachable:
                self.get_logger().info(
                    f"❌ Obje elendi: {obj.class_name} "
                    f"x={obj.x:.2f}, y={obj.y:.2f}, z={obj.z:.2f}, sebep={result.reason}"
                )
                continue

            valid.append(obj)

        return valid

    def select_next_object(self):
        valid_objects = self.filter_reachable_objects()

        if not valid_objects:
            return None

        # Önce x küçük, sonra merkeze yakın y, sonra confidence yüksek
        valid_objects.sort(
            key=lambda obj: (
                obj.x,
                abs(obj.y),
                -obj.confidence
            )
        )

        return valid_objects[0]

    def send_target(self, point, target_type):
        now = time.time()

        if now - self.last_target_sent_time < self.min_seconds_between_targets:
            return False

        self.target_pub.publish(point)
        self.last_target_sent_time = now
        self.current_target_type = target_type

        self.get_logger().info(
            f"🚀 Hedef gönderildi [{target_type}] -> "
            f"x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}"
        )

        return True

    def get_class_target(self, class_name):
        if class_name in self.class_targets:
            return self.class_targets[class_name]

        # Bilinmeyen sınıf için plastik hedefine gönder
        self.get_logger().warn(
            f"⚠️ {class_name} için hedef yok. Plastik hedefi kullanılacak."
        )
        return self.class_targets["plastik"]

    def handle_motion_success(self):
        if self.state == "WAIT_OBJECT_REACHED":
            self.state = "GO_TO_CLASS_TARGET"

        elif self.state == "WAIT_CLASS_TARGET_REACHED":
            self.state = "GO_HOME"

        elif self.state == "WAIT_HOME_REACHED":
            if self.current_object is not None:
                self.done_targets.append(self.object_to_point(self.current_object))
                self.get_logger().info(
                    f"🏁 Obje tamamlandı ve done listesine eklendi: "
                    f"x={self.current_object.x:.2f}, y={self.current_object.y:.2f}, z={self.current_object.z:.2f}"
                )

            self.current_object = None
            self.current_target_type = None
            self.state = "SELECT_OBJECT"

    def handle_motion_failed(self):
        # Hareket başarısızsa o objeyi atla, sistem takılı kalmasın.
        if self.current_object is not None:
            self.done_targets.append(self.object_to_point(self.current_object))
            self.get_logger().warn("⚠️ Başarısız obje geçici olarak done listesine alındı.")

        self.current_object = None
        self.current_target_type = None
        self.state = "SELECT_OBJECT"

    def check_motion_timeout(self):
        if self.state not in [
            "WAIT_OBJECT_REACHED",
            "WAIT_CLASS_TARGET_REACHED",
            "WAIT_HOME_REACHED"
        ]:
            return

        elapsed = time.time() - self.last_target_sent_time
        if elapsed > self.motion_timeout_sec:
            self.get_logger().warn(
                f"⏱️ Hareket timeout: {self.current_target_type}. Sıradaki adıma geçiliyor."
            )
            self.handle_motion_failed()

    def state_loop(self):
        self.publish_status(self.state)
        self.check_motion_timeout()

        if self.state == "WAIT_OBJECTS":
            if self.detected_objects:
                self.state = "SELECT_OBJECT"

        elif self.state == "SELECT_OBJECT":
            selected = self.select_next_object()

            if selected is None:
                self.state = "WAIT_OBJECTS"
                return

            self.current_object = selected
            self.get_logger().info(
                f"🎯 Obje seçildi: {selected.class_name} "
                f"x={selected.x:.2f}, y={selected.y:.2f}, z={selected.z:.2f}, conf={selected.confidence:.2f}"
            )
            self.state = "GO_TO_OBJECT"

        elif self.state == "GO_TO_OBJECT":
            if self.current_object is None:
                self.state = "SELECT_OBJECT"
                return

            point = self.object_to_point(self.current_object)
            sent = self.send_target(point, "object")

            if sent:
                self.state = "WAIT_OBJECT_REACHED"

        elif self.state == "GO_TO_CLASS_TARGET":
            if self.current_object is None:
                self.state = "SELECT_OBJECT"
                return

            class_target = self.get_class_target(self.current_object.class_name)

            result = self.reachability.is_reachable(
                class_target.x,
                class_target.y,
                class_target.z
            )

            if not result.reachable:
                self.get_logger().warn(
                    f"⚠️ Sınıf hedefi ulaşılmaz: {self.current_object.class_name}, sebep={result.reason}"
                )
                self.handle_motion_failed()
                return

            sent = self.send_target(class_target, f"class_target_{self.current_object.class_name}")

            if sent:
                self.state = "WAIT_CLASS_TARGET_REACHED"

        elif self.state == "GO_HOME":
            result = self.reachability.is_reachable(
                self.home_target.x,
                self.home_target.y,
                self.home_target.z
            )

            if not result.reachable:
                self.get_logger().warn(f"⚠️ HOME ulaşılmaz: {result.reason}. Home atlanıyor.")
                if self.current_object is not None:
                    self.done_targets.append(self.object_to_point(self.current_object))
                self.current_object = None
                self.state = "SELECT_OBJECT"
                return

            sent = self.send_target(self.home_target, "home")

            if sent:
                self.state = "WAIT_HOME_REACHED"


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()