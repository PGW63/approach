cmake_minimum_required(VERSION 3.8)
project(approach_map_runner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FAST_GICP_LOCAL_PREFIX
  "${CMAKE_CURRENT_SOURCE_DIR}/../fast_gicp/local_install"
)
set(FAST_GICP_LOCAL_LIB_DIR
  "${FAST_GICP_LOCAL_PREFIX}/lib"
)

if(EXISTS "${FAST_GICP_LOCAL_PREFIX}/share/fast_gicp/cmake/fast_gicpConfig.cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${FAST_GICP_LOCAL_PREFIX}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(approach_icp REQUIRED)
find_package(approach_mapping REQUIRED)
find_package(approach_cost REQUIRED)
find_package(approach_preprocess REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(inha_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(approach_map_runner_node
  src/approach_map_runner_node.cpp
)

add_executable(approach_cost_runner_node
  src/approach_cost_runner_node.cpp
)

add_executable(node_icp_cuda
  src/node_icp_cuda.cpp
)

target_include_directories(approach_map_runner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(approach_cost_runner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(node_icp_cuda PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(approach_map_runner_node
  approach_mapping::approach_map_lib
  approach_preprocess::approach_preprocess_lib
  yaml-cpp
)

target_link_libraries(approach_cost_runner_node
  approach_mapping::approach_map_lib
  approach_cost::approach_cost_lib
  yaml-cpp
)

target_link_libraries(node_icp_cuda
  approach_icp::approach_icp_core
  approach_preprocess::approach_preprocess_lib
  yaml-cpp
)

if(EXISTS "${FAST_GICP_LOCAL_LIB_DIR}")
  set_target_properties(node_icp_cuda PROPERTIES
    BUILD_RPATH "${FAST_GICP_LOCAL_LIB_DIR}"
    INSTALL_RPATH "${FAST_GICP_LOCAL_LIB_DIR}"
  )
endif()

ament_target_dependencies(approach_map_runner_node
  ament_index_cpp
  approach_mapping
  approach_preprocess
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  inha_interfaces
)

ament_target_dependencies(approach_cost_runner_node
  ament_index_cpp
  approach_mapping
  approach_cost
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  inha_interfaces
)

ament_target_dependencies(node_icp_cuda
  ament_index_cpp
  approach_icp
  approach_preprocess
  fast_gicp
  geometry_msgs
  inha_interfaces
  pcl_conversions
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  approach_map_runner_node
  approach_cost_runner_node
  node_icp_cuda
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
