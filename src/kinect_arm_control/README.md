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
├── config
├── description
│   └── urdf
│       └── my_custom_robot.urdf
├── images
├── launch
│   └── start_system.launch.py
├── resource
│   └── kinect_arm_control
├── scripts
│   ├── detector_color.py
│   ├── detector_yolo.py
│   ├── __init__.py
│   └── robot_brain.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── weights
├── package.xml
├── README.md
├── setup.cfg
└── setup.py
src/my_custom_arm_moveit_config
├── config
│   ├── custom_robot_arm_rect.ros2_control.xacro
│   ├── custom_robot_arm_rect.srdf
│   ├── custom_robot_arm_rect.urdf.xacro
│   ├── initial_positions.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   ├── moveit.rviz
│   ├── ompl_planning.yaml
│   ├── pilz_cartesian_limits.yaml
│   └── ros2_controllers.yaml
├── launch
│   ├── demo.launch.py
│   ├── gazebo.launch.py
│   ├── move_group.launch.py
│   ├── moveit_planning.launch.py
│   ├── moveit_rviz.launch.py
│   ├── rsp.launch.py
│   ├── setup_assistant.launch.py
│   ├── spawn_controllers.launch.py
│   ├── static_virtual_joint_tfs.launch.py
│   └── warehouse_db.launch.py
├── CMakeLists.txt
└── package.xml

2 directories, 23 files

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
ros2 launch kinect_arm_control start_system.launch.py
```

Opsiyonel Argümanlar: Simülasyon yerine gerçek robotla çalışmak isterseniz:
```bash

ros2 launch kinect_arm_control start_system.launch.py use_sim_time:=false use_gazebo:=false
```

Geliştirici: [Mofxel1]
