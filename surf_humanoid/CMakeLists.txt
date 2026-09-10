cmake_minimum_required(VERSION 3.8)
project(surf_humanoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(surf_multirobot_comms REQUIRED)
find_package(surf_multirobot_msgs REQUIRED)

add_executable(drone_data_receiver src/drone_data_receiver.cpp)
ament_target_dependencies(
  drone_data_receiver rclcpp surf_multirobot_comms surf_multirobot_msgs)

install(TARGETS drone_data_receiver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sync_recovery test/test_sync_recovery.cpp TIMEOUT 60)
  ament_target_dependencies(test_sync_recovery rclcpp surf_multirobot_comms surf_multirobot_msgs)
endif()
ament_package()
