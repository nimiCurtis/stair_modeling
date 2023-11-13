cmake_minimum_required(VERSION 3.5)
project(stair_modeling)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#############################################
# Dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(zion_msgs REQUIRED)
find_package(zion_components REQUIRED)

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_CURRENT_SOURCE_DIR}/include/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

###############################################################################
# SOURCES

set(ZION_BROADCASTER_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/zion_broadcaster_node.cpp
)


set(STAIR_MODELING_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling.cpp
)

###############################################################################
# Bin and Install

add_executable(stair_modeling
  ${STAIR_MODELING_SRC}
)
ament_target_dependencies(stair_modeling
  rclcpp
  rclcpp_components
  zion_components
)
install(TARGETS stair_modeling
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(zion_broadcaster
  ${ZION_BROADCASTER_SRC}
)
ament_target_dependencies(zion_broadcaster
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_msgs
)
install(TARGETS
  zion_broadcaster
  DESTINATION lib/${PROJECT_NAME})


# Install config/launch/rviz2 ... files
install(DIRECTORY
  config
  launch
  rviz2
  DESTINATION share/${PROJECT_NAME}
)


###############################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
