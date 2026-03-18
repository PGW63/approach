cmake_minimum_required(VERSION 3.8)
project(approach)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search kdtree)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObject.srv"
)

set(APPROACH_CPP_SOURCES
  src/main.cpp
  src/roi_filter.cpp
)

add_executable(approach_node ${APPROACH_CPP_SOURCES})
target_include_directories(approach_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_compile_features(approach_node PUBLIC cxx_std_17)
target_link_libraries(approach_node
  ${PCL_LIBRARIES}
)
target_compile_definitions(approach_node
  PRIVATE
    ${PCL_DEFINITIONS}
)
ament_target_dependencies(approach_node
  geometry_msgs
  message_filters
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  vision_msgs
  visualization_msgs
)

install(
  TARGETS approach_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/configs/params.yaml
  DESTINATION share/${PROJECT_NAME}/configs
)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  message_filters
  pcl_conversions
  rclcpp
  rosidl_default_runtime
  sensor_msgs
  tf2
  tf2_ros
  vision_msgs
  visualization_msgs
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
