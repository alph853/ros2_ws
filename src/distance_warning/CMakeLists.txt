cmake_minimum_required(VERSION 3.8)
project(distance_warning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(distance_warning_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Include directories
include_directories(include)


# =============================================================================
# C++ EXECUTABLES (DEFAULT)
# =============================================================================

# Distance publisher executable
add_executable(distance_publisher
  src/distance_publisher.cpp
)
ament_target_dependencies(distance_publisher
  rclcpp
  distance_warning_interfaces
  rclcpp_action
)

# Distance listener executable
add_executable(distance_listener
  src/distance_listener.cpp
)
ament_target_dependencies(distance_listener
  rclcpp
  distance_warning_interfaces
  rclcpp_action
)

# Set threshold service executable
add_executable(set_threshold_service
  src/set_threshold_service.cpp
)
ament_target_dependencies(set_threshold_service
  rclcpp
  distance_warning_interfaces
  rclcpp_action
)

# Distance action server executable
add_executable(distance_action_server
  src/distance_action_server.cpp
)
ament_target_dependencies(distance_action_server
  rclcpp
  rclcpp_action
  distance_warning_interfaces
)

# Distance action client executable
add_executable(distance_action_client
  src/distance_action_client.cpp
)
ament_target_dependencies(distance_action_client
  rclcpp
  rclcpp_action
  distance_warning_interfaces
)

# Install C++ executables
install(TARGETS
  distance_publisher
  distance_listener
  set_threshold_service
  distance_action_server
  distance_action_client
  DESTINATION lib/${PROJECT_NAME}
)


find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/distance_publisher.py
  scripts/distance_listener.py
  scripts/set_threshold_service.py
  scripts/distance_action_server.py
  scripts/distance_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# COMMON INSTALLATIONS
# =============================================================================

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
