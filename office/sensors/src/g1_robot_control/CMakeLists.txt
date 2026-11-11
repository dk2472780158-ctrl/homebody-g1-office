cmake_minimum_required(VERSION 3.8)
project(g1_robot_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(g1_loco_client_example src/loco_client_example.cpp)
target_include_directories(g1_loco_client_example PRIVATE include include/common include/nlohmann)
ament_target_dependencies(g1_loco_client_example unitree_api rclcpp)

add_executable(g1_cmd_vel_bridge src/cmd_vel_bridge.cpp)
target_include_directories(g1_cmd_vel_bridge PRIVATE include include/common include/nlohmann)
ament_target_dependencies(g1_cmd_vel_bridge unitree_api rclcpp geometry_msgs)

install(TARGETS g1_loco_client_example g1_cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
