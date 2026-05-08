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
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)

include(ExternalProject)

set(ZIP_URL "https://github.com/bluerobotics/navigator-lib/releases/latest/download/cpp.zip")
set(NAVIGATOR_DIR "${CMAKE_BINARY_DIR}/navigator_zip_contents")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7")
  set(NAVIGATOR_ARCH "armv7-unknown-linux-gnueabihf")
  set(TARGET_RASPBERRY TRUE)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(NAVIGATOR_ARCH "aarch64-unknown-linux-gnu")
  set(TARGET_RASPBERRY TRUE)
else()
  message(WARNING "Building sura_sensors in desktop mode on unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  set(TARGET_RASPBERRY FALSE)
endif()

if(TARGET_RASPBERRY)
  ExternalProject_Add(
    navigator_lib
    URL ${ZIP_URL}
    DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/navigator_download"
    SOURCE_DIR ${NAVIGATOR_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    TEST_COMMAND ""
  )

  set(NAVIGATOR_LIB "${NAVIGATOR_DIR}/${NAVIGATOR_ARCH}/libbluerobotics_navigator.so")
endif()

# Nodos
add_executable(battery_node
  src/battery_node.cpp
)

add_executable(gps_node
  src/gps_node.cpp
)

ament_target_dependencies(battery_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(gps_node
  rclcpp
  sensor_msgs
)

# Broadcasters
add_library(${PROJECT_NAME} SHARED
  src/broadcasters/magnetometer_broadcaster.cpp
  src/broadcasters/pressure_broadcaster.cpp
  src/broadcasters/dvl75_vel_broadcaster.cpp
  src/broadcasters/dvl75_distance_broadcaster.cpp 
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

if(TARGET_RASPBERRY)
  add_dependencies(battery_node navigator_lib)

  target_include_directories(battery_node PRIVATE
    include
    ${NAVIGATOR_DIR}
  )

  target_link_libraries(battery_node
    ${NAVIGATOR_LIB}
  )

  target_compile_definitions(battery_node PRIVATE TARGET_RASPBERRY)

  target_include_directories(gps_node PRIVATE
    include
  )
  target_compile_definitions(gps_node PRIVATE TARGET_RASPBERRY)

else()
  target_include_directories(battery_node PRIVATE
    include
  )
  target_compile_definitions(battery_node PRIVATE TARGET_DESKTOP)

  target_include_directories(gps_node PRIVATE
    include
  )
  target_compile_definitions(gps_node PRIVATE TARGET_DESKTOP)
endif()

install(TARGETS
  battery_node
  gps_node
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