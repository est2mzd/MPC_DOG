cmake_minimum_required(VERSION 3.8)
project(recovery_controller)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/recovery_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME recovery_controller_node
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    recovery_controller_tests
    test/test_recovery_controller.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    TIMEOUT 30
  )
endif()

ament_package()
