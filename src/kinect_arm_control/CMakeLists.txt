cmake_minimum_required(VERSION 3.8)
project(kinect_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# GEREKLI PAKETLER
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

# ---------------------------------------------------------
# C++ BEYINLER
# ---------------------------------------------------------

# Eski sabit beyin
add_executable(cpp_brain src/cpp_brain.cpp)
ament_target_dependencies(cpp_brain
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  sensor_msgs
)

# Mevcut dinamik beyin
add_executable(dynamic_brain_node src/dynamic_brain_node.cpp)
ament_target_dependencies(dynamic_brain_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  sensor_msgs
  urdf
)

# Yeni queue/task-manager uyumlu dinamik beyin
add_executable(dynamic_brain_node_queue src/dynamic_brain_node_queue.cpp)
ament_target_dependencies(dynamic_brain_node_queue
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  sensor_msgs
  std_msgs
  urdf
)

# ---------------------------------------------------------
# KURULUM (INSTALL) AYARLARI
# ---------------------------------------------------------

# 1. Launch dosyalarını yükle
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. URDF ve mesh dosyalarını yükle
install(DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

# 3. Python executable scriptleri yükle
install(PROGRAMS
  scripts/detector_yolo.py
  scripts/detector_yolo_multi.py
  scripts/task_manager_node.py
  scripts/teleop_terminal.py
  scripts/stm_bridge.py
  scripts/add_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Python import dosyalarını yükle
install(FILES
  scripts/reachability.py
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Derlenen C++ node'ları yükle
install(TARGETS
  cpp_brain
  dynamic_brain_node
  dynamic_brain_node_queue
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()