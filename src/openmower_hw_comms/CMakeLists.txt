cmake_minimum_required(VERSION 3.8)
project(openmower_hw_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/openmower_hw_comms.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN openmower_hw_comms::OpenMowerHWComms
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_auto_package(
#   INSTALL_TO_SHARE
#     launch
#     params
# )
