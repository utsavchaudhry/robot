cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config (RViz)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts (joint_state_relay, ik_interactive_marker, publish_robot_description, ik_keyboard_controller, teleop_data_logger, record_flicker)
install(PROGRAMS
  scripts/joint_state_relay.py
  scripts/ik_interactive_marker_node.py
  scripts/publish_robot_description.py
  scripts/ik_keyboard_controller.py
  scripts/teleop_data_logger.py
  scripts/slider_command_relay.py
  scripts/record_flicker.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
