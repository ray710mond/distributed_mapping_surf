cmake_minimum_required(VERSION 3.8)
project(surf_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs QUIET)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(surf_multirobot_comms REQUIRED)
find_package(surf_multirobot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

add_library(drone_communication_core src/adaptive_mode.cpp)
target_include_directories(drone_communication_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(drone_communication_core PUBLIC surf_multirobot_msgs)

add_executable(drone_scan_sender src/drone_scan_sender.cpp)
ament_target_dependencies(drone_scan_sender
  geometry_msgs nav_msgs rclcpp sensor_msgs surf_multirobot_comms surf_multirobot_msgs
  tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(drone_scan_sender drone_communication_core Threads::Threads)

set(surf_drone_targets drone_communication_core drone_scan_sender)
if(px4_msgs_FOUND)
  add_executable(px4_state_bridge src/px4_state_bridge.cpp)
  ament_target_dependencies(px4_state_bridge
    diagnostic_msgs geometry_msgs nav_msgs px4_msgs rclcpp sensor_msgs tf2)
  list(APPEND surf_drone_targets px4_state_bridge)
else()
  message(WARNING
    "px4_msgs was not found; building surf_drone without px4_state_bridge. "
    "Source a matching px4_msgs workspace and rebuild to enable PX4 support.")
endif()

install(TARGETS ${surf_drone_targets}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_adaptive_mode test/test_adaptive_mode.cpp)
  if(TARGET test_adaptive_mode)
    target_link_libraries(test_adaptive_mode drone_communication_core)
    ament_target_dependencies(test_adaptive_mode surf_multirobot_msgs)
  endif()
endif()
ament_package()
