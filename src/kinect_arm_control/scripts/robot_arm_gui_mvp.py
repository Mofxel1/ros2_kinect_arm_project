#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Kol Operatör GUI - 3D Dijital İkiz + Atık Ayrıştırma Paneli

Bu arayüz teknik debug ekranı değil, işi bilmeyen operatörün kullanabileceği sade tesis panelidir.

Ana özellikler:
- URDF/STL tabanlı 3D dijital ikiz
- Kamera görüntüsü paneli
- ROS2 /joint_states dinleyerek dijital ikizi hareket ettirme
- Robotu Durdur / Home Pozisyonuna Git komutları
- Anlık seçilen atık bilgisi
- Bugün ayrıştırılan atık sayıları
- Canlı bar grafik
- Arıza durumu göstergesi
- Teknik olmayan operatör olay akışı

ROS2 beklenen topicler:

1) Joint açıları:
    Topic: /joint_states
    Type : sensor_msgs/msg/JointState
    name:     ["joint0", "joint1", "joint2"]
    position: [0.0, 0.4, -0.3]   # radyan

2) GUI komutları:
    Topic: /robot_gui_command
    Type : std_msgs/msg/String
    data : "stop" veya "home"

3) Anlık seçilen/tespit edilen atık:
    Topic: /selected_waste
    Type : std_msgs/msg/String
    data : "plastic", "metal", "clear_glass", "green_glass", "brown_glass"

4) Başarıyla ayrıştırılan atık:
    Topic: /waste_sorted
    Type : std_msgs/msg/String
    data : "plastic", "metal", "clear_glass", "green_glass", "brown_glass"

5) Arıza durumu:
    Topic: /system_fault
    Type : std_msgs/msg/String
    data : "none" veya "clear" normal durum kabul edilir.
           Bunun dışındaki her değer arıza olarak gösterilir.

Çalıştırma:
    cd ~/ros2_ws/src/kinect_arm_control/scripts

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

    python3 robot_arm_gui_mvp.py \
        --urdf ../description/urdf/robot_montaj.urdf \
        --camera-index 1 \
        --ros2
"""

import argparse
import math
import os
import re
import sys
import tempfile
import time
from dataclasses import dataclass
from typing import Dict, List, Optional


def fix_qt_plugin_path():
    candidate_paths = [
        "/usr/lib/x86_64-linux-gnu/qt5/plugins",
        "/usr/lib/qt/plugins",
    ]
    for path in candidate_paths:
        if os.path.exists(os.path.join(path, "platforms", "libqxcb.so")):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = path
            break
    os.environ.pop("QT_PLUGIN_PATH", None)


fix_qt_plugin_path()

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_msgs.msg import Float32MultiArray
    ROS2_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    JointState = None
    String = None
    Float32MultiArray = None
    ROS2_AVAILABLE = False

import numpy as np
import pybullet as p
import pybullet_data
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


def get_cv2():
    import cv2
    return cv2


@dataclass
class JointLimit:
    lower: float
    upper: float


WASTE_LABELS = {
    "plastic": "Plastik",
    "plastik": "Plastik",
    "metal": "Metal",
    "clear_glass": "Şeffaf Cam",
    "seffaf_cam": "Şeffaf Cam",
    "şeffaf_cam": "Şeffaf Cam",
    "green_glass": "Yeşil Cam",
    "yesil_cam": "Yeşil Cam",
    "yeşil_cam": "Yeşil Cam",
    "brown_glass": "Kahverengi Cam",
    "kahverengi_cam": "Kahverengi Cam",
}

WASTE_KEYS = ["plastic", "metal", "clear_glass", "green_glass", "brown_glass"]
WASTE_DISPLAY = {
    "plastic": "Plastik",
    "metal": "Metal",
    "clear_glass": "Şeffaf Cam",
    "green_glass": "Yeşil Cam",
    "brown_glass": "Kahverengi Cam",
}

# YOLO/detected_objects sınıf ID eşleşmesi.
# /detected_objects örnek data formatı: [x, y, z, confidence, class_id]
CLASS_ID_TO_WASTE = {
    0: "plastic",
    1: "metal",
    2: "clear_glass",
    3: "green_glass",
    4: "brown_glass",
}


def normalize_waste_key(raw: str) -> str:
    value = str(raw).strip().lower().replace(" ", "_").replace("-", "_")
    aliases = {
        "plastik": "plastic",
        "plastic": "plastic",
        "metal": "metal",
        "cam_seffaf": "clear_glass",
        "seffaf_cam": "clear_glass",
        "şeffaf_cam": "clear_glass",
        "clear_glass": "clear_glass",
        "cam_yesil": "green_glass",
        "yesil_cam": "green_glass",
        "yeşil_cam": "green_glass",
        "green_glass": "green_glass",
        "cam_kahverengi": "brown_glass",
        "kahverengi_cam": "brown_glass",
        "brown_glass": "brown_glass",
    }
    return aliases.get(value, value)


def waste_to_display(raw: str) -> str:
    key = normalize_waste_key(raw)
    return WASTE_DISPLAY.get(key, WASTE_LABELS.get(str(raw).strip().lower(), str(raw)))


class URDFMeshResolver:
    PACKAGE_URI_PATTERN = re.compile(r'filename="package://([^"]+)"')

    @staticmethod
    def resolve_urdf(urdf_path: str) -> str:
        urdf_path = os.path.abspath(urdf_path)
        urdf_dir = os.path.dirname(urdf_path)
        with open(urdf_path, "r", encoding="utf-8") as file:
            text = file.read()

        def replace_match(match):
            package_relative_path = match.group(1)
            filename = os.path.basename(package_relative_path)
            candidates = [
                os.path.join(urdf_dir, filename),
                os.path.join(urdf_dir, "meshes", filename),
                os.path.join(os.path.dirname(urdf_dir), "meshes", filename),
            ]
            parts = package_relative_path.split("/")
            if len(parts) >= 2:
                package_name = parts[0]
                inner_path = os.path.join(*parts[1:])
                ros_roots = [
                    os.path.expanduser("~/ros2_ws/src"),
                    os.path.expanduser("~/catkin_ws/src"),
                    os.getcwd(),
                ]
                ros_package_path = os.environ.get("ROS_PACKAGE_PATH", "")
                for root in ros_package_path.split(os.pathsep):
                    if root:
                        ros_roots.append(root)
                for root in ros_roots:
                    candidates.append(os.path.join(root, package_name, inner_path))

            for search_root in [urdf_dir, os.path.dirname(urdf_dir)]:
                if os.path.exists(search_root):
                    for root, _, files in os.walk(search_root):
                        if filename in files:
                            candidates.append(os.path.join(root, filename))
                            break

            for candidate in candidates:
                if os.path.exists(candidate):
                    return f'filename="{candidate}"'
            return match.group(0)

        resolved_text = URDFMeshResolver.PACKAGE_URI_PATTERN.sub(replace_match, text)
        temp_dir = tempfile.mkdtemp(prefix="robot_gui_urdf_")
        resolved_urdf_path = os.path.join(temp_dir, os.path.basename(urdf_path))
        with open(resolved_urdf_path, "w", encoding="utf-8") as file:
            file.write(resolved_text)
        return resolved_urdf_path


class PyBulletRobotScene:
    def __init__(self):
        self.client_id: Optional[int] = None
        self.robot_id: Optional[int] = None
        self.joint_name_to_index: Dict[str, int] = {}
        self.joint_limits: Dict[str, JointLimit] = {}
        self.active_joint_names: List[str] = []
        self.camera_yaw = 45
        self.camera_pitch = -25
        self.camera_distance = 1.2
        self.camera_target = [0.1, 0.0, 0.18]
        self.resolved_urdf_path: Optional[str] = None

    def connect(self):
        if self.client_id is not None:
            return
        self.client_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        p.loadURDF("plane.urdf", physicsClientId=self.client_id)

    def load_robot(self, urdf_path: str):
        self.connect()
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF bulunamadı: {urdf_path}")
        if self.robot_id is not None:
            p.removeBody(self.robot_id, physicsClientId=self.client_id)
        self.resolved_urdf_path = URDFMeshResolver.resolve_urdf(urdf_path)
        self.robot_id = p.loadURDF(
            self.resolved_urdf_path,
            basePosition=[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.client_id,
        )
        self.extract_joint_info()
        self.reset_default_pose()

    def extract_joint_info(self):
        self.joint_name_to_index.clear()
        self.joint_limits.clear()
        self.active_joint_names.clear()
        joint_count = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        for idx in range(joint_count):
            info = p.getJointInfo(self.robot_id, idx, physicsClientId=self.client_id)
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            lower = float(info[8])
            upper = float(info[9])
            self.joint_name_to_index[joint_name] = idx
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                if lower > upper:
                    lower, upper = -math.pi, math.pi
                self.joint_limits[joint_name] = JointLimit(lower=lower, upper=upper)
                self.active_joint_names.append(joint_name)

    def reset_default_pose(self):
        if self.robot_id is None:
            return
        for joint_name in self.active_joint_names:
            idx = self.joint_name_to_index[joint_name]
            limit = self.joint_limits.get(joint_name, JointLimit(-math.pi, math.pi))
            target = 0.0 if limit.lower <= 0.0 <= limit.upper else (limit.lower + limit.upper) / 2.0
            p.resetJointState(self.robot_id, idx, target, physicsClientId=self.client_id)

    def set_joint_value_rad(self, joint_name: str, value_rad: float):
        if self.robot_id is None or joint_name not in self.joint_name_to_index:
            return
        idx = self.joint_name_to_index[joint_name]
        limit = self.joint_limits.get(joint_name, JointLimit(-math.pi, math.pi))
        value_rad = max(limit.lower, min(limit.upper, value_rad))
        p.resetJointState(self.robot_id, idx, value_rad, physicsClientId=self.client_id)

    def render(self, width=640, height=480):
        if self.client_id is None:
            self.connect()
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.camera_target,
            distance=self.camera_distance,
            yaw=self.camera_yaw,
            pitch=self.camera_pitch,
            roll=0,
            upAxisIndex=2,
        )
        proj = p.computeProjectionMatrixFOV(
            fov=55,
            aspect=width / height,
            nearVal=0.01,
            farVal=10.0,
        )
        _, _, rgba, _, _ = p.getCameraImage(
            width,
            height,
            viewMatrix=view,
            projectionMatrix=proj,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=self.client_id,
        )
        rgba_np = np.reshape(rgba, (height, width, 4)).astype(np.uint8)
        return rgba_np[:, :, :3]

    def disconnect(self):
        if self.client_id is not None:
            p.disconnect(self.client_id)
            self.client_id = None
            self.robot_id = None


class Ros2GuiNode(Node):
    def __init__(self, joint_topic: str, command_topic: str, selected_topic: str, sorted_topic: str, fault_topic: str, detected_topic: str):
        super().__init__("robot_arm_gui_node")
        self.latest_positions_rad: Dict[str, float] = {}
        self.pending_selected: List[str] = []
        self.pending_sorted: List[str] = []
        self.latest_fault: Optional[str] = None

        self.create_subscription(JointState, joint_topic, self.joint_state_callback, 10)
        self.create_subscription(String, selected_topic, self.selected_callback, 10)
        self.create_subscription(String, sorted_topic, self.sorted_callback, 10)
        self.create_subscription(String, fault_topic, self.fault_callback, 10)
        self.create_subscription(Float32MultiArray, detected_topic, self.detected_objects_callback, 10)
        self.command_publisher = self.create_publisher(String, command_topic, 10)

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.latest_positions_rad[name] = float(position)

    def selected_callback(self, msg):
        self.pending_selected.append(str(msg.data))

    def detected_objects_callback(self, msg):
        data = list(msg.data)
        if not data:
            return

        # Beklenen format: [x, y, z, confidence, class_id]
        # Birden fazla obje ardışık gelirse 5'li paketler halinde işler.
        packet_size = 5
        if len(data) >= packet_size and len(data) % packet_size == 0:
            packets = [data[i:i + packet_size] for i in range(0, len(data), packet_size)]
        else:
            packets = [data]

        for packet in packets:
            class_id = int(round(packet[-1]))
            waste_key = CLASS_ID_TO_WASTE.get(class_id)
            if waste_key is not None:
                self.pending_selected.append(waste_key)

    def sorted_callback(self, msg):
        self.pending_sorted.append(str(msg.data))

    def fault_callback(self, msg):
        self.latest_fault = str(msg.data)

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)


class ImageLabel(QLabel):
    def __init__(self, text="", parent=None):
        super().__init__(parent)
        self.setMinimumSize(500, 350)
        self.setAlignment(Qt.AlignCenter)
        self.setText(text)
        self.setStyleSheet(
            "background-color: #020617; color: #e5e7eb; border-radius: 12px; font-size: 16px;"
        )

    def update_rgb(self, frame_rgb):
        if frame_rgb is None:
            return
        frame_rgb = np.ascontiguousarray(frame_rgb, dtype=np.uint8)
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w
        q_image = QImage(frame_rgb.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def update_bgr(self, frame_bgr):
        if frame_bgr is None:
            return
        cv2 = get_cv2()
        self.update_rgb(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))


class WasteBarChart(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.counts = {key: 0 for key in WASTE_KEYS}
        self.setMinimumHeight(210)
        self.setStyleSheet("background-color: #f8fafc; border-radius: 10px;")

    def set_counts(self, counts: Dict[str, int]):
        self.counts = dict(counts)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()
        margin_left = 110
        margin_right = 25
        margin_top = 25
        bar_height = 24
        gap = 13

        painter.fillRect(self.rect(), QColor("#f8fafc"))
        painter.setFont(QFont("Arial", 10))

        max_value = max(max(self.counts.values()), 1)
        available_width = width - margin_left - margin_right

        for i, key in enumerate(WASTE_KEYS):
            label = WASTE_DISPLAY[key]
            value = int(self.counts.get(key, 0))
            y = margin_top + i * (bar_height + gap)
            bar_width = int((value / max_value) * available_width) if value > 0 else 0

            painter.setPen(QColor("#334155"))
            painter.drawText(10, y + 17, label)

            painter.setBrush(QColor("#dbeafe"))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(QRectF(margin_left, y, available_width, bar_height), 6, 6)

            painter.setBrush(QColor("#2563eb"))
            painter.drawRoundedRect(QRectF(margin_left, y, max(bar_width, 2 if value > 0 else 0), bar_height), 6, 6)

            painter.setPen(QColor("#0f172a"))
            painter.drawText(margin_left + available_width - 45, y + 17, str(value))

        painter.end()


class RobotArmMainWindow(QMainWindow):
    def __init__(
        self,
        urdf_path: Optional[str],
        camera_index: int = 0,
        use_ros2: bool = False,
        joint_topic: str = "/joint_states",
        command_topic: str = "/robot_gui_command",
        selected_topic: str = "/selected_waste",
        sorted_topic: str = "/waste_sorted",
        fault_topic: str = "/system_fault",
        detected_topic: str = "/detected_objects",
    ):
        super().__init__()
        self.setWindowTitle("Akıllı Atık Ayrıştırma Kontrol Paneli")
        self.resize(1450, 880)

        self.urdf_path = urdf_path
        self.scene = PyBulletRobotScene()
        self.use_ros2 = use_ros2
        self.joint_topic = joint_topic
        self.command_topic = command_topic
        self.selected_topic = selected_topic
        self.sorted_topic = sorted_topic
        self.fault_topic = fault_topic
        self.detected_topic = detected_topic
        self.ros_node = None
        self.ros_initialized = False

        self.camera = None
        self.camera_index = camera_index
        self.camera_running = False
        self.render_error_logged = False

        self.waste_counts = {
            "plastic": 48,
            "metal": 26,
            "clear_glass": 19,
            "green_glass": 21,
            "brown_glass": 14,
        }
        self.count_labels: Dict[str, QLabel] = {}
        self.current_fault = "none"

        self.digital_twin_view = ImageLabel("3D robot modeli hazırlanıyor")
        self.camera_view = ImageLabel("Kamera görüntüsü bekleniyor")

        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMinimumHeight(240)
        self.event_log.setStyleSheet(
            "background-color: #f8fafc; color: #0f172a; border: 1px solid #cbd5e1; border-radius: 8px; padding: 8px; font-size: 14px;"
        )

        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.update_3d_view)
        self.render_timer.start(33)

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_frame)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.update_ros2_data)

        self.build_ui()
        self.load_robot_if_possible()
        self.setup_ros2_if_requested()
        self.refresh_dashboard()
        self.seed_demo_operator_events()

    def build_ui(self):
        root = QWidget()
        main_layout = QVBoxLayout(root)

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.digital_twin_view, stretch=1)
        top_layout.addWidget(self.camera_view, stretch=1)

        dashboard_layout = QHBoxLayout()
        dashboard_layout.addWidget(self.create_status_group(), stretch=1)
        dashboard_layout.addWidget(self.create_chart_group(), stretch=2)

        command_group = QGroupBox("Operatör Komutları")
        command_layout = QHBoxLayout(command_group)

        self.stop_button = QPushButton("Robotu Durdur")
        self.stop_button.clicked.connect(self.send_stop_command)
        self.home_button = QPushButton("Home Pozisyonuna Git")
        self.home_button.clicked.connect(self.send_home_command)
        self.camera_button = QPushButton("Kamerayı Başlat")
        self.camera_button.clicked.connect(self.toggle_camera)

        self.style_button(self.stop_button, "#dc2626", "#b91c1c")
        self.style_button(self.home_button, "#16a34a", "#15803d")
        self.style_button(self.camera_button, "#2563eb", "#1d4ed8")

        command_layout.addWidget(self.stop_button)
        command_layout.addWidget(self.home_button)
        command_layout.addWidget(self.camera_button)

        event_group = QGroupBox("Operatör Olay Akışı")
        event_layout = QVBoxLayout(event_group)
        event_layout.addWidget(self.event_log)

        main_layout.addLayout(top_layout, stretch=2)
        main_layout.addLayout(dashboard_layout, stretch=1)
        main_layout.addWidget(event_group, stretch=1)
        main_layout.addWidget(command_group)
        self.setCentralWidget(root)

    def style_button(self, button: QPushButton, color: str, hover: str):
        button.setMinimumHeight(44)
        button.setStyleSheet(
            f"QPushButton {{ background-color: {color}; color: white; border-radius: 8px; font-weight: bold; padding: 8px; font-size: 15px; }}"
            f"QPushButton:hover {{ background-color: {hover}; }}"
        )

    def create_status_group(self):
        group = QGroupBox("Anlık Durum")
        layout = QVBoxLayout(group)
        self.system_status_label = QLabel("Sistem hazırlanıyor")
        self.current_selection_label = QLabel("Seçilen atık: Bekleniyor")
        self.fault_status_label = QLabel("Arıza durumu: Kontrol ediliyor")
        self.total_count_label = QLabel("Bugün ayrıştırılan toplam atık: 0")
        for label in [self.system_status_label, self.current_selection_label, self.fault_status_label, self.total_count_label]:
            label.setMinimumHeight(35)
            label.setStyleSheet("font-size: 16px; color: #0f172a;")
        layout.addWidget(self.system_status_label)
        layout.addWidget(self.current_selection_label)
        layout.addWidget(self.fault_status_label)
        layout.addWidget(self.total_count_label)
        return group

    def create_count_group(self):
        group = QGroupBox("Bugünkü Ayrıştırma Verisi")
        layout = QGridLayout(group)
        self.total_count_label = QLabel("Toplam: 0")
        self.total_count_label.setStyleSheet("font-size: 22px; font-weight: bold; color: #2563eb;")
        layout.addWidget(self.total_count_label, 0, 0, 1, 2)
        self.count_labels: Dict[str, QLabel] = {}
        for i, key in enumerate(WASTE_KEYS):
            label = QLabel(f"{WASTE_DISPLAY[key]}: 0")
            label.setStyleSheet("font-size: 15px; color: #0f172a;")
            self.count_labels[key] = label
            layout.addWidget(label, 1 + i // 2, i % 2)
        return group

    def create_chart_group(self):
        group = QGroupBox("Canlı Atık Grafiği")
        layout = QVBoxLayout(group)
        self.waste_chart = WasteBarChart()
        layout.addWidget(self.waste_chart)
        return group

    def seed_demo_operator_events(self):
        """Sunum/demo için arayüz ilk açıldığında örnek işlem akışı gösterir."""
        demo_events = [
            "Kamera görüntüsü aktif hale getirildi.",
            "Bant üzerindeki atıklar taranıyor.",

            "Plastik atık tespit edildi.",
            "Plastik atık ayrıştırma için seçildi.",
            "Robot plastik atığa yönlendiriliyor.",
            "Plastik atık kavrandı.",
            "Plastik atık ilgili kutuya bırakıldı.",
            "Plastik sayacı güncellendi. Toplam plastik: 48.",
            "Sistem yeni atık için hazır.",

            "Metal atık tespit edildi.",
            "Metal atık ayrıştırma için seçildi.",
            "Robot metal atığa yönlendiriliyor.",
            "Metal atık kavrandı.",
            "Metal atık ilgili kutuya bırakıldı.",
            "Metal sayacı güncellendi. Toplam metal: 26.",
            "Sistem yeni atık için hazır.",

            "Şeffaf cam atık tespit edildi.",
            "Şeffaf cam atık ayrıştırma için seçildi.",
            "Robot şeffaf cam atığa yönlendiriliyor.",
            "Şeffaf cam atık kavrandı.",
            "Şeffaf cam atık ilgili kutuya bırakıldı.",
            "Şeffaf cam sayacı güncellendi. Toplam şeffaf cam: 19.",
            "Sistem yeni atık için hazır.",

            "Yeşil cam atık tespit edildi.",
            "Yeşil cam atık ayrıştırma için seçildi.",
            "Robot yeşil cam atığa yönlendiriliyor.",
            "Yeşil cam atık kavrandı.",
            "Yeşil cam atık ilgili kutuya bırakıldı.",
            "Yeşil cam sayacı güncellendi. Toplam yeşil cam: 21.",
            "Sistem yeni atık için hazır.",

            "Kahverengi cam atık tespit edildi.",
            "Kahverengi cam atık ayrıştırma için seçildi.",
            "Robot kahverengi cam atığa yönlendiriliyor.",
            "Kahverengi cam atık kavrandı.",
            "Kahverengi cam atık ilgili kutuya bırakıldı.",
            "Kahverengi cam sayacı güncellendi. Toplam kahverengi cam: 14.",
            "Sistem normal şekilde çalışmaya devam ediyor.",
        ]
        for event in demo_events:
            self.add_operator_event(event)

    def setup_ros2_if_requested(self):
        if not self.use_ros2:
            self.add_operator_event("Sistem görüntüleme modunda açıldı. ROS2 bağlantısı kapalı.")
            self.set_system_ok("Sistem görüntüleme modunda")
            return
        if not ROS2_AVAILABLE:
            self.add_operator_event("ROS2 bağlantısı başlatılamadı. Terminal kaynak ayarları kontrol edilmeli.")
            self.set_fault("ROS2 bağlantısı yok")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.ros_initialized = True
            self.ros_node = Ros2GuiNode(
                self.joint_topic,
                self.command_topic,
                self.selected_topic,
                self.sorted_topic,
                self.fault_topic,
                self.detected_topic,
            )
            self.ros_timer.start(20)
            self.add_operator_event("Sistem bağlantısı kuruldu. Robot verileri izleniyor.")
            self.set_system_ok("Sistem aktif")
        except Exception:
            self.add_operator_event("Sistem bağlantısı başlatılamadı. Yetkiliye haber veriniz.")
            self.set_fault("ROS2 bağlantı hatası")

    def load_robot_if_possible(self):
        if not self.urdf_path:
            self.add_operator_event("Robot modeli bulunamadı. Yetkiliye haber veriniz.")
            self.set_fault("Robot modeli yok")
            return
        try:
            self.scene.load_robot(self.urdf_path)
            self.add_operator_event("Robot dijital ikizi hazır.")
        except Exception:
            self.add_operator_event("Robot modeli yüklenemedi. Yetkiliye haber veriniz.")
            self.set_fault("Robot modeli yüklenemedi")

    def send_stop_command(self):
        self.publish_gui_command("stop", "Robot durduruldu.")

    def send_home_command(self):
        self.publish_gui_command("home", "Robot home pozisyonuna gönderildi.")

    def publish_gui_command(self, command: str, message: str):
        if self.use_ros2 and self.ros_node is not None:
            try:
                self.ros_node.publish_command(command)
                self.add_operator_event(message)
                return
            except Exception:
                self.add_operator_event("Komut gönderilemedi. Yetkiliye haber veriniz.")
                self.set_fault("Komut gönderme hatası")
                return
        self.add_operator_event("Komut gönderilemedi. Sistem bağlantısı kapalı.")

    def update_ros2_data(self):
        if not self.use_ros2 or self.ros_node is None:
            return
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
            self.update_joint_states_from_ros()
            self.update_waste_events_from_ros()
            self.update_fault_from_ros()
        except Exception:
            self.add_operator_event("Sistem verileri okunamadı. Yetkiliye haber veriniz.")
            self.set_fault("Veri okuma hatası")
            self.ros_timer.stop()

    def update_joint_states_from_ros(self):
        latest = self.ros_node.latest_positions_rad
        if not latest:
            return
        for joint_name in self.scene.active_joint_names:
            if joint_name in latest:
                self.scene.set_joint_value_rad(joint_name, latest[joint_name])

    def update_waste_events_from_ros(self):
        while self.ros_node.pending_selected:
            raw = self.ros_node.pending_selected.pop(0)
            display = waste_to_display(raw)
            self.current_selection_label.setText(f"Seçilen atık: {display}")
            self.add_operator_event(f"{display} seçildi.")

        while self.ros_node.pending_sorted:
            raw = self.ros_node.pending_sorted.pop(0)
            key = normalize_waste_key(raw)
            display = waste_to_display(raw)
            if key in self.waste_counts:
                self.waste_counts[key] += 1
            self.add_operator_event(f"{display} ayrıştırıldı.")
            self.refresh_dashboard()

    def update_fault_from_ros(self):
        fault = self.ros_node.latest_fault
        if fault is None:
            return
        fault_value = fault.strip().lower()
        if fault_value in ["none", "clear", "ok", "normal", ""]:
            if self.current_fault != "none":
                self.add_operator_event("Arıza durumu giderildi. Sistem normal çalışıyor.")
            self.current_fault = "none"
            self.set_system_ok("Sistem normal çalışıyor")
        else:
            if self.current_fault != fault_value:
                self.add_operator_event(f"Arıza bildirildi: {self.friendly_fault_name(fault_value)}")
            self.current_fault = fault_value
            self.set_fault(self.friendly_fault_name(fault_value))

    def friendly_fault_name(self, fault: str) -> str:
        names = {
            "camera_error": "Kamera bağlantısı kontrol edilmeli",
            "robot_error": "Robot hareketi kontrol edilmeli",
            "conveyor_error": "Konveyör sistemi kontrol edilmeli",
            "emergency_stop": "Acil durdurma aktif",
            "yolo_error": "Görüntü işleme kontrol edilmeli",
        }
        return names.get(fault, fault.replace("_", " "))

    def refresh_dashboard(self):
        total = sum(self.waste_counts.values())
        self.total_count_label.setText(f"Bugün ayrıştırılan toplam atık: {total}")
        for key, label in self.count_labels.items():
            label.setText(f"{WASTE_DISPLAY[key]}: {self.waste_counts.get(key, 0)}")
        self.waste_chart.set_counts(self.waste_counts)

    def set_system_ok(self, message: str):
        self.system_status_label.setText(message)
        self.system_status_label.setStyleSheet("font-size: 16px; color: #15803d; font-weight: bold;")
        self.fault_status_label.setText("Arıza durumu: Yok")
        self.fault_status_label.setStyleSheet("font-size: 16px; color: #15803d;")

    def set_fault(self, message: str):
        self.system_status_label.setText("Sistem uyarı durumunda")
        self.system_status_label.setStyleSheet("font-size: 16px; color: #dc2626; font-weight: bold;")
        self.fault_status_label.setText(f"Arıza durumu: {message}")
        self.fault_status_label.setStyleSheet("font-size: 16px; color: #dc2626; font-weight: bold;")

    def update_3d_view(self):
        try:
            frame_rgb = self.scene.render(640, 480)
            self.digital_twin_view.update_rgb(frame_rgb)
        except Exception:
            if not self.render_error_logged:
                self.add_operator_event("Robot görüntüsü hazırlanamadı. Yetkiliye haber veriniz.")
                self.render_error_logged = True

    def toggle_camera(self):
        if self.camera_running:
            self.stop_camera()
        else:
            self.start_camera()

    def start_camera(self):
        cv2 = get_cv2()
        self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        if not self.camera.isOpened():
            self.add_operator_event("Kamera açılamadı. Kamera bağlantısı kontrol edilmeli.")
            self.set_fault("Kamera bağlantısı kontrol edilmeli")
            self.camera = None
            return
        self.camera_running = True
        self.camera_button.setText("Kamerayı Durdur")
        self.camera_timer.start(33)
        self.add_operator_event("Kamera görüntüsü başlatıldı.")

    def stop_camera(self):
        self.camera_timer.stop()
        if self.camera is not None:
            self.camera.release()
            self.camera = None
        self.camera_running = False
        self.camera_button.setText("Kamerayı Başlat")
        self.camera_view.setText("Kamera durduruldu")
        self.add_operator_event("Kamera görüntüsü durduruldu.")

    def update_camera_frame(self):
        if self.camera is None:
            return
        ok, frame = self.camera.read()
        if not ok:
            self.add_operator_event("Kamera görüntüsü alınamadı. Kamera bağlantısı kontrol edilmeli.")
            self.set_fault("Kamera görüntüsü alınamıyor")
            return
        self.camera_view.update_bgr(frame)

    def add_operator_event(self, message: str):
        timestamp = time.strftime("%H:%M:%S")
        self.event_log.append(f"[{timestamp}] {message}")

    def closeEvent(self, event):
        self.stop_camera()
        self.ros_timer.stop()
        if self.ros_node is not None:
            try:
                self.ros_node.destroy_node()
            except Exception:
                pass
        if self.ros_initialized and rclpy is not None:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        self.scene.disconnect()
        event.accept()


def parse_args():
    parser = argparse.ArgumentParser(description="Robot Kol Operatör GUI")
    parser.add_argument("--urdf", type=str, default=None, help="Robot URDF dosya yolu")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV kamera index değeri")
    parser.add_argument("--ros2", action="store_true", help="ROS2 bağlantısını aç")
    parser.add_argument("--joint-topic", type=str, default="/joint_states", help="JointState topic adı")
    parser.add_argument("--command-topic", type=str, default="/robot_gui_command", help="GUI komut topic adı")
    parser.add_argument("--selected-topic", type=str, default="/selected_waste", help="Seçilen atık topic adı")
    parser.add_argument("--sorted-topic", type=str, default="/waste_sorted", help="Ayrıştırılan atık topic adı")
    parser.add_argument("--fault-topic", type=str, default="/system_fault", help="Arıza topic adı")
    parser.add_argument("--detected-topic", type=str, default="/detected_objects", help="Float32MultiArray tespit topic adı")
    return parser.parse_args()


def main():
    args = parse_args()
    app = QApplication(sys.argv)
    window = RobotArmMainWindow(
        urdf_path=args.urdf,
        camera_index=args.camera_index,
        use_ros2=args.ros2,
        joint_topic=args.joint_topic,
        command_topic=args.command_topic,
        selected_topic=args.selected_topic,
        sorted_topic=args.sorted_topic,
        fault_topic=args.fault_topic,
        detected_topic=args.detected_topic,
    )
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
