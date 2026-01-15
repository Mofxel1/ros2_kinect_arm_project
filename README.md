# 🤖 ROS 2 Robot Arm Control with Kinect & MoveIt

Bu proje, **ROS 2 Humble** üzerinde çalışan otonom bir robot kol simülasyonudur. Sistem, **Kinect kamera** verilerini kullanarak yeşil bir nesneyi (topu) tespit eder, 3B konumunu hesaplar ve **MoveIt** kullanarak robot kolunu nesneye yönlendirir.

![ROS 2 Badge](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Python Badge](https://img.shields.io/badge/Language-Python3-yellow.svg)
![MoveIt Badge](https://img.shields.io/badge/Motion-MoveIt-orange.svg)

## 🚀 Özellikler

* **Görüntü İşleme (OpenCV):** RGB ve Derinlik (Depth) verilerini birleştirerek nesne tespiti.
* **Hareket Planlama (MoveIt 2):** Engellerden kaçınan güvenli yol planlaması.
* **Ters Kinematik (IK):** Hedef koordinatlar için gerekli eklem açılarının matematiksel hesaplaması.
* **Akıllı Kontrol:** Robot sadece hedef belirli bir eşiğin (1.5 cm) üzerinde yer değiştirdiğinde hareket eder (Gereksiz titremeyi önler).
* **ROS 2 Control:** `FakeSystem` donanım arayüzü ile gerçekçi motor simülasyonu.

## 🛠️ Kullanılan Teknolojiler

* **ROS 2 Humble** (Ubuntu 22.04)
* **MoveIt 2** (Setup Assistant & Move Group)
* **OpenCV** (Görüntü İşleme)
* **CV Bridge** (ROS-OpenCV bağlantısı)
* **Xacro/URDF** (Robot Modelleme)

## 📦 Kurulum

Bu projeyi kendi bilgisayarınızda çalıştırmak için aşağıdaki adımları izleyin:

1. **Çalışma Alanını Oluşturun:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. **Depoyu Klonlayın:**
    ```bash
    git clone [https://github.com/Mofxel1/ros2_kinect_arm_project.git](https://github.com/Mofxel1/ros2_kinect_arm_project.git) .
    ```

3. **Gerekli Paketleri Yükleyin:**
    ```bash
    sudo apt update
    sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-cv-bridge
    ```

4. **Derleyin:**
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

## ▶️ Nasıl Çalıştırılır?

Sistemi çalıştırmak için 3 farklı terminal açmanız gerekmektedir.

**Terminal 1: Simülasyon ve MoveIt**
Robotu, RViz arayüzünü ve MoveIt planlama sistemini başlatır.
source install/setup.bash
ros2 launch my_custom_arm_moveit_config demo.launch.py

Terminal 2: Kamera Sürücüsü Kinect kamerasını (veya simülasyonunu) başlatır.
source install/setup.bash
ros2 run kinect_ros2 kinect_ros2_node

Terminal 3: Robot Beyni (Otonom Kontrol) Görüntüyü işleyen ve robotu yöneten ana Python düğümünü başlatır.
source install/setup.bash
ros2 run kinect_arm_control start
