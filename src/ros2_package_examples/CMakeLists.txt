cmake_minimum_required(VERSION 3.8)
project(ros2_package_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(action_msgs REQUIRED)

# Generate actions
add_action_files(
  DIRECTORY action
  FILES
  RobotCommand.action
)

generate_messages(
  DEPENDENCIES
  action_msgs
  std_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  ros2_package_examples/simple_node.py
  ros2_package_examples/talker_node.py
  ros2_package_examples/listener_node.py
  ros2_package_examples/joint_controller.py
  ros2_package_examples/advanced_joint_controller.py
  ros2_package_examples/robot_executor_server.py
  ros2_package_examples/llm_client.py
  ros2_package_examples/llm_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()