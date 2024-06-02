cmake_minimum_required(VERSION 3.8)
project(open_mower_next)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

###
# map_server_node
###
add_executable(map_server_node
        src/map_server/node_main.cpp
        src/map_server/map_server_node.hpp
        src/map_server/map_server_node.cpp
        src/map_server/geo_json_map.cpp
        src/map_server/geo_json_map.hpp
        src/map_server/polygon_iterator.hpp
        src/map_server/some_gaussian_filter.hpp
        src/map_server/polygon_utils.hpp)
target_compile_features(map_server_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(map_server_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(map_server_node
        std_msgs
        geometry_msgs
        nav_msgs
        foxglove_msgs
        visualization_msgs
        rclcpp
        robot_localization
        tf2
        tf2_geometry_msgs
)

target_link_libraries(map_server_node nlohmann_json::nlohmann_json)

INSTALL(TARGETS map_server_node
        DESTINATION lib/${PROJECT_NAME})

##
# map_server_node
##

###
# sim_node
###
add_executable(sim_node
        src/sim/node_main.cpp
        src/sim/sim_node.hpp
        src/sim/sim_node.cpp
        src/sim/sim_node.cpp
        src/sim/sim_node.hpp
        src/sim/node_main.cpp)
target_compile_features(sim_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(sim_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sim_node
        rclcpp
        tf2
        tf2_geometry_msgs
)

INSTALL(TARGETS sim_node
        DESTINATION lib/${PROJECT_NAME})

##
# sim_node
##

rosidl_generate_interfaces(${PROJECT_NAME}
        "src/srv/RemoveArea.srv"
        "src/srv/RemoveDockingStation.srv"
        "src/srv/SaveArea.srv"
        "src/srv/SaveDockingStation.srv"
        "src/msg/Area.msg"
        "src/msg/Map.msg"
        "src/msg/DockingStation.msg"
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        tf2
        tf2_geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(map_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(sim_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

INSTALL(
        DIRECTORY config launch description worlds maps
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
