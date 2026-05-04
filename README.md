# 🤖 ROS 2 Robot Arm Control with Kinect & MoveIt

Bu proje, **ROS 2 Humble** üzerinde çalışan otonom bir robot kol simülasyonudur. Sistem, **Kinect kamera** verilerini kullanarak nesne tespiti yapar (YOLO/Renk), 3B konumunu hesaplar ve **MoveIt** kullanarak robot kolunu nesneye yönlendirir.

![ROS 2 Badge](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Python Badge](https://img.shields.io/badge/Language-Python3-yellow.svg)
![MoveIt Badge](https://img.shields.io/badge/Motion-MoveIt-orange.svg)

## 🎥 Proje Demosu (Gerçek vs Simülasyon)

Proje, **Dijital İkiz (Digital Twin)** mantığıyla çalışmaktadır. Simülasyon ortamında hesaplanan yörüngeler, gerçek robota eş zamanlı olarak aktarılır.

| 🦾 Gerçek Robot (Real World) | 🖥️ Simülasyon (Gazebo/RViz) |
| :---: | :---: |
| ![Gerçek Robot](src/kinect_arm_control/images/Kol1.gif) | ![Simülasyon](src/kinect_arm_control/images/kol2.gif) |

## 🚀 Özellikler

* **Orkestra Şefi (Launch System):** Tek bir komutla Gazebo, MoveIt, RViz ve Yapay Zeka node'larını senkronize şekilde başlatır.
* **Görüntü İşleme (YOLO & OpenCV):** RGB ve Derinlik (Depth) verilerini birleştirerek hassas nesne tespiti.
* **Hareket Planlama (MoveIt 2):** Engellerden kaçınan güvenli yol planlaması.
* **Modüler Yapı:** Konfigürasyon ve kod birbirinden ayrılmıştır (`params.yaml` ile yönetim).
* **ROS 2 Control:** `FakeSystem` donanım arayüzü ile gerçekçi motor simülasyonu.
* **Çok Objeli Görev Akışı:** Algılanan birden fazla obje sıraya alınır; robot önce objeye, sonra sınıfına göre belirlenen bırakma noktasına gider.

## 📂 Proje Yapısı

Projenin temel dosya ağacı aşağıdadır:

```text
src/kinect_arm_control
├── CMakeLists.txt
├── description
│   ├── meshes
│   │   ├── base_link.STL
│   │   ├── link_1.STL
│   │   ├── link_2.STL
│   │   └── link_3.STL
│   └── urdf
│       ├── my_custom_robot.urdf
│       ├── robot_montaj.csv
│       └── robot_montaj.urdf
├── images
│   ├── Kol1.gif
│   └── kol2.gif
├── launch
│   ├── debug_brain.launch.py
│   ├── debug_brain_queue.launch.py
│   ├── manual_control.launch.py
│   ├── multi_object_bringup.launch.py
│   ├── start_system.launch.py
│   └── system_bringup.launch.py
├── package.xml
├── resource
│   └── kinect_arm_control
├── scripts
│   ├── add_obstacle.py
│   ├── detector_color.py
│   ├── detector_yolo.py
│   ├── detector_yolo_multi.py
│   ├── eye_node_mobile.py
│   ├── eye_node.py
│   ├── __init__.py
│   ├── reachability.py
│   ├── robot_brain.py
│   ├── stm_bridge.py
│   ├── task_manager_node.py
│   ├── teleop_terminal.py
│   └── yolov8n.pt
├── src
│   ├── cpp_brain.cpp
│   ├── dynamic_brain_node.cpp
│   └── dynamic_brain_node_queue.cpp
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
src/my_custom_arm_moveit_config
├── CMakeLists.txt
├── config
│   ├── initial_positions.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   ├── moveit.rviz
│   ├── pilz_cartesian_limits.yaml
│   ├── robot_montaj.ros2_control.xacro
│   ├── robot_montaj.srdf
│   ├── robot_montaj.urdf.xacro
│   ├── ros2_controllers.yaml
│   └── sensors_3d.yaml
├── launch
│   ├── demo.launch.py
│   ├── move_group.launch.py
│   ├── moveit_rviz.launch.py
│   ├── rsp.launch.py
│   ├── setup_assistant.launch.py
│   ├── spawn_controllers.launch.py
│   ├── static_virtual_joint_tfs.launch.py
│   └── warehouse_db.launch.py
└── package.xml

2 directories, 21 files
```

## 🛠️ Kullanılan Teknolojiler

    ROS 2 Humble (Ubuntu 22.04)

    MoveIt 2 (Setup Assistant & Move Group)

    OpenCV (Görüntü İşleme)

    YOLO (Nesne Algılama)

    Xacro/URDF (Robot Modelleme)

## 📦 Kurulum

Bu projeyi kendi bilgisayarınızda çalıştırmak için aşağıdaki adımları izleyin:

Çalışma Alanını Oluşturun:
    
```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
```

Depoyu Klonlayın:
```bash
    git clone https://github.com/Mofxel1/ros2_kinect_arm_project.git .
```

(Not: Eğer sadece kaynak kodları alacaksanız kinect_arm_control ve my_custom_arm_moveit_config klasörlerini almanız yeterlidir.)

Gerekli Bağımlılıkları Yükleyin:
    
```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-cv-bridge
pip install ultralytics  # YOLO için
```
Derleyin:
    
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## ▶️ Nasıl Çalıştırılır?

Projeyi çalıştırmak için karmaşık terminal işlemlerine gerek yoktur. Hazırlanan merkezi launch dosyası tüm sistemi sırasıyla (Gazebo -> MoveIt -> AI) başlatır.

Tek Komutla Başlatma (Önerilen):

```bash
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src
ros2 launch kinect_arm_control system_bringup.launch.py
```

Çok Objeli Görev Sistemi:

```bash
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src
ros2 launch kinect_arm_control multi_object_bringup.launch.py
```

Kamera Olmadan Konum Gönderme:

```bash
ros2 topic pub -1 /camera/target_coords geometry_msgs/msg/Point "{x: 0.40, y: 0.0, z: 0.2}"
```

Kamera Olmadan Çok Objeli Sisteme Test Objesi Gönderme:

```bash
ros2 topic pub --once /detected_objects std_msgs/msg/Float32MultiArray "{data: [0.30, 0.0, 0.10, 0.90, 0.0]}"
```

Bu format şu şekildedir:

```text
[x, y, z, confidence, class_id]
```

Örnek olarak yukarıdaki komut:

```text
x = 0.30 m
 y = 0.00 m
 z = 0.10 m
 confidence = 0.90
 class_id = 0  # plastik
```

## 🧠 Çok Objeli Görev Mantığı

Yeni görev akışında kamera bütün objeleri algılar, `task_manager_node.py` bu objeleri sıraya alır ve robot koluna tek tek hedef gönderir.

```text
YOLO / Kinect
    ↓
/detected_objects
    ↓
task_manager_node.py
    ↓
/camera/target_coords
    ↓
dynamic_brain_node_queue.cpp
    ↓
MoveIt + Gazebo/RViz
    ↓
/arm_motion_done
```

Temel hareket sırası:

```text
1. Obje konumuna git
2. Objenin sınıfına göre belirlenen bırakma noktasına git
3. Home / bekleme konumuna dön
4. Sıradaki objeye geç
```

## ⚠️ Notlar

* Eğer Gazebo açılmazsa arkada eski Gazebo süreci kalmış olabilir. Temizlemek için:

```bash
killall -9 gazebo gzserver gzclient rviz2 robot_state_publisher move_group spawn_entity.py 2>/dev/null
ros2 daemon stop
ros2 daemon start
```

* `best.pt` gibi YOLO model dosyaları büyük olabilir. GitHub 50 MB üstü dosyalar için uyarı verir. İleride bu dosyalar için Git LFS kullanılabilir.

Geliştirici: [Mofxel1]
