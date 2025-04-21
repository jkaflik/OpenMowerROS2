###
# sim_node
###
add_executable(sim_node
        src/sim/node_main.cpp
        src/sim/sim_node.cpp)
target_compile_features(sim_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(sim_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(sim_node "${cpp_typesupport_target}")

ament_target_dependencies(sim_node
        rclcpp
        tf2
        tf2_geometry_msgs
)

INSTALL(TARGETS sim_node
        DESTINATION lib/${PROJECT_NAME})

add_dependencies(sim_node ${PROJECT_NAME})