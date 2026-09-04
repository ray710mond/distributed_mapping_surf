cmake_minimum_required(VERSION 3.8)
project(surf_multirobot_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(surf_multirobot_msgs REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)

add_library(${PROJECT_NAME} SHARED src/voxel_codec.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp surf_multirobot_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC zstd::zstd)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp surf_multirobot_msgs zstd_vendor zstd)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxel_codec test/test_voxel_codec.cpp)
  if(TARGET test_voxel_codec)
    target_link_libraries(test_voxel_codec ${PROJECT_NAME})
  endif()
endif()

ament_package()
