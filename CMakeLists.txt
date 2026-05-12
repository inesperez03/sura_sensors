cmake_minimum_required(VERSION 3.8)
project(sura_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)

# Nodes
add_executable(gps_node
  src/gps_node.cpp
)

add_executable(gps_anchor_node
  src/gps_anchor_node.cpp
)

ament_target_dependencies(gps_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(gps_anchor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  std_msgs
)

# Broadcasters
add_library(${PROJECT_NAME} SHARED
  src/broadcasters/imu_broadcaster.cpp
  src/broadcasters/magnetometer_broadcaster.cpp
  src/broadcasters/pressure_broadcaster.cpp
  src/broadcasters/battery_broadcaster.cpp
  src/broadcasters/gps_broadcaster.cpp
  src/broadcasters/dvl75_vel_broadcaster.cpp
  src/broadcasters/dvl75_altitude_broadcaster.cpp
  src/broadcasters/dvl75_gps_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface
  sura_sensors_plugins.xml
)

target_include_directories(gps_node PRIVATE
  include
)

target_include_directories(gps_anchor_node PRIVATE
  include
)

install(TARGETS
  gps_node
  gps_anchor_node
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES sura_sensors_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
