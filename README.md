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
│   ├── manual_control.launch.py
│   ├── start_system.launch.py
│   └── system_bringup.launch.py
├── package.xml
├── resource
│   └── kinect_arm_control
├── scripts
│   ├── detector_color.py
│   ├── detector_yolo.py
│   ├── eye_node_mobile.py
│   ├── eye_node.py
│   ├── __init__.py
│   ├── robot_brain.py
│   ├── teleop_terminal.py
│   └── yolov8n.pt
├── src
│   └── cpp_brain.cpp
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


Geliştirici: [Mofxel1]
