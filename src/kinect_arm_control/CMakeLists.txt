cmake_minimum_required(VERSION 3.8)
project(kinect_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Gerekli Paketleri Bul
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++ Beynini (cpp_brain) Derle
add_executable(cpp_brain src/cpp_brain.cpp)
ament_target_dependencies(cpp_brain
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  sensor_msgs
)

# YENI DINAMIK BEYIN ICIN DERLEME KURALLARI
add_executable(dynamic_brain_node src/dynamic_brain_node.cpp)
ament_target_dependencies(dynamic_brain_node 
  rclcpp 
  moveit_ros_planning_interface 
  geometry_msgs
  moveit_msgs 
  sensor_msgs 
  urdf
)
# ---------------------------------------------------------
# KURULUM (INSTALL) AYARLARI
# ---------------------------------------------------------

# 1. Launch Dosyalarını Yükle
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. URDF ve Mesh Dosyalarını Yükle (Kritik!)
install(DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

# 3. Python Scriptini (YOLO) Yükle (Kritik!)
# Scriptin "scripts" klasöründe olduğunu varsayıyoruz.
install(PROGRAMS 
  scripts/detector_yolo.py
  scripts/teleop_terminal.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Derlenen C++ Node'unu Yükle
install(TARGETS 
  cpp_brain
  dynamic_brain_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
