cmake_minimum_required(VERSION 3.8)
project(openmower)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)

add_executable(openmower_datum_publisher_node
        src/datum_publisher/node.cpp
        src/datum_publisher/node.hpp
        src/datum_publisher/node_main.cpp
)

ament_target_dependencies(openmower_datum_publisher_node
        rclcpp
        std_msgs
        sensor_msgs
        geographic_msgs
        robot_localization
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

INSTALL(
  DIRECTORY config launch description worlds maps
  DESTINATION share/${PROJECT_NAME}
)

# Installation and export info
install(TARGETS openmower_datum_publisher_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
