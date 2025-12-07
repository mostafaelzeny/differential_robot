cmake_minimum_required(VERSION 3.8)
project(robot_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(a_star_planner src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner rclcpp std_msgs nav_msgs geometry_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs)

add_executable(dijkstra_planner src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp std_msgs nav_msgs geometry_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs)

add_executable(pd_motion_planner src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner rclcpp std_msgs nav_msgs geometry_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs)

install(TARGETS
  a_star_planner
  dijkstra_planner
  pd_motion_planner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
