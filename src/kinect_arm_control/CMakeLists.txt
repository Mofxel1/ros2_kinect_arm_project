cmake_minimum_required(VERSION 3.8)
project(kinect_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++ Node'u Derle
add_executable(cpp_brain src/cpp_brain.cpp)
ament_target_dependencies(cpp_brain
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  sensor_msgs
)

# --- DEĞİŞİKLİK BURADA ---
# "config" klasörü olmadığı için onu listeden çıkardık.
# Sadece "launch" klasörünü kopyalıyoruz.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Executable'ı yükle
install(TARGETS cpp_brain
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
