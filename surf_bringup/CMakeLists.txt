cmake_minimum_required(VERSION 3.8)
project(surf_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lio_config test/test_lio_config.py)
  ament_add_pytest_test(test_prepare_map test/test_prepare_map.py)
  ament_add_pytest_test(test_scanlock_handoff test/test_scanlock_handoff.py)
endif()

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(PROGRAMS scripts/prepare_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME prepare_map
)
install(PROGRAMS scripts/slam_map_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/analyze_mid360_bag.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME analyze_mid360_bag
)
install(PROGRAMS scripts/localization_overlap_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME localization_overlap_monitor
)
install(PROGRAMS scripts/slam_to_scanlock_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slam_to_scanlock_supervisor
)
install(PROGRAMS scripts/live_occupancy_projector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME live_occupancy_projector
)
install(PROGRAMS scripts/peer_clock_guard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME peer_clock_guard
)

ament_package()
