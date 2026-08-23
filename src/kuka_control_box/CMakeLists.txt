cmake_minimum_required(VERSION 3.8)
project(kuka_control_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kuka_control_box_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
include_directories(include)
# Robotics 라이브러리 생성
add_library(Robotics src/Robotics.cpp)
ament_target_dependencies(Robotics rclcpp Eigen3)
# kuka_ik_node 실행 파일 설정
add_executable(kuka_ik_node src/kuka_ik_node.cpp)
target_link_libraries(kuka_ik_node Robotics)
ament_target_dependencies(kuka_ik_node 
  rclcpp 
  Eigen3 
  kuka_control_box_srvs 
  nav_msgs 
  geometry_msgs 
  trajectory_msgs
  tf2
  tf2_ros  # 추가
  tf2_geometry_msgs  # 추가
  sensor_msgs
  )
# 설치 경로 설정
install(TARGETS
  kuka_ik_node
  Robotics
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
ament_package()
