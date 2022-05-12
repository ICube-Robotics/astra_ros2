cmake_minimum_required(VERSION 3.5)
project(astra_ros2)

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

set(ASTRA_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/external/astra_sdk)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/astra_sdk/samples/cmake)  

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Astra REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

# add_executable(depth_camera_node_cpp nodes/depth_camera_node_cpp.cpp)
add_executable(astra_node nodes/astra_node.cpp )
target_link_libraries(astra_node ${ASTRA_LIBRARIES})
ament_target_dependencies(astra_node rclcpp sensor_msgs geometry_msgs)

add_executable(surface_node nodes/surface_node.cpp )
ament_target_dependencies(surface_node rclcpp sensor_msgs pcl_ros pcl_conversions visualization_msgs shape_msgs Eigen3)

include_directories(${ASTRA_INCLUDE_DIR})

install(TARGETS
astra_node
surface_node
  DESTINATION lib/${PROJECT_NAME})

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
