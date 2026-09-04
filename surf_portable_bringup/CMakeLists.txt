cmake_minimum_required(VERSION 3.8)
project(surf_portable_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/global_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME global_odometry)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_portable_launch test/test_portable_launch.py)
endif()

ament_package()
