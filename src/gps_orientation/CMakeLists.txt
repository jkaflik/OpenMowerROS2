cmake_minimum_required(VERSION 3.8)
include(ExternalProject)

project(gps_orientation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(gps_orientation_node
        src/node.cpp
        src/node.hpp
        src/node_main.cpp
        src/core.hpp
        src/system_model.hpp
        src/core.cpp
)

set(EP_KALMAN "Kalman")
ExternalProject_Add(
        ${EP_KALMAN}

        PREFIX ${EP_KALMAN}
        GIT_REPOSITORY https://github.com/mherb/kalman
        GIT_TAG origin/master
        GIT_SHALLOW OFF

        BUILD_ALWAYS OFF

        INSTALL_COMMAND cmake -E echo "Skipping install step."

        CMAKE_CACHE_ARGS
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release
)

add_dependencies(gps_orientation_node ${EP_KALMAN})
ExternalProject_Get_property(${EP_KALMAN} SOURCE_DIR)
message("Source dir of ${EP_KALMAN} = ${SOURCE_DIR}")
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
        include
        ${SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(
        gps_orientation_node
        ${EIGEN3_LIBRARIES}
)

ament_target_dependencies(gps_orientation_node
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        tf2
        tf2_geometry_msgs
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

install(TARGETS gps_orientation_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
