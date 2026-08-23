cmake_minimum_required(VERSION 3.8)
project(kuka_control_box_srvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)  # TF2 의존성 추가
find_package(tf2_ros REQUIRED)  # TF2 ROS 의존성 추가
find_package(tf2_geometry_msgs REQUIRED)  # tf2_geometry_msgs 의존성 추가

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KukaTaskInput.srv"
  "srv/KukaTask.srv"
  "srv/KukaJoint.srv"
  "srv/KukaTransformInput.srv"  # 서비스 정의
  DEPENDENCIES geometry_msgs  # 메시지 의존성 설정
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(tf2)  # TF2 의존성 내보내기
ament_export_dependencies(tf2_ros)  # TF2 ROS 의존성 내보내기
ament_export_dependencies(tf2_geometry_msgs)  # tf2_geometry_msgs 의존성 내보내기

ament_package()
