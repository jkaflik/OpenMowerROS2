cmake_minimum_required(VERSION 3.8)
project(open_mower_map_server)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(map_server_node
        src/node_main.cpp
        include/open_mower_map_server/map_server_node.hpp
        src/map_server_node.cpp
        src/geo_json_map.cpp
        include/open_mower_map_server/geo_json_map.hpp
        include/open_mower_map_server/polygon_iterator.hpp
        include/open_mower_map_server/some_gaussian_filter.hpp
        include/open_mower_map_server/polygon_utils.hpp
        include/open_mower_map_server/polygon_utils.hpp)
target_include_directories(map_server_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(map_server_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Area.msg"
        "msg/Map.msg"
        "msg/DockingStation.msg"
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_geometry_msgs
)

ament_target_dependencies(map_server_node
        std_msgs
        geometry_msgs
        nav_msgs
        rclcpp
        robot_localization
        tf2
        tf2_geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(map_server_node "${cpp_typesupport_target}")
target_link_libraries(map_server_node nlohmann_json::nlohmann_json)

install(TARGETS map_server_node
        DESTINATION lib/${PROJECT_NAME})

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

ament_package()
