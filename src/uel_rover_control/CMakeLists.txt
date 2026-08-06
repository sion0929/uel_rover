cmake_minimum_required(VERSION 3.8)
project(uel_rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# ── 노드 빌드 ────────────────────────────────────────────────────
add_executable(odom_publisher src/odom_publisher.cpp)

ament_target_dependencies(odom_publisher rclcpp nav_msgs std_msgs tf2_ros tf2 geometry_msgs)

# ── 설치 ─────────────────────────────────────────────────────────
install(TARGETS
  odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
