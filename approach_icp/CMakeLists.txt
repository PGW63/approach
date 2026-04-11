cmake_minimum_required(VERSION 3.8)
project(approach_icp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration)
find_package(geometry_msgs REQUIRED)
find_package(inha_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(FAST_GICP_LOCAL_PREFIX
  "${CMAKE_CURRENT_SOURCE_DIR}/../fast_gicp/local_install"
)
set(FAST_GICP_LOCAL_LIB_DIR
  "${FAST_GICP_LOCAL_PREFIX}/lib"
)

if(EXISTS "${FAST_GICP_LOCAL_PREFIX}/share/fast_gicp/cmake/fast_gicpConfig.cmake")
  message(STATUS "Using local fast_gicp from ${FAST_GICP_LOCAL_PREFIX}")
  find_package(fast_gicp REQUIRED CONFIG
    PATHS "${FAST_GICP_LOCAL_PREFIX}"
    NO_DEFAULT_PATH
  )
else()
  find_package(fast_gicp REQUIRED)
endif()

add_library(${PROJECT_NAME}_core
  src/performance_metrics.cpp
  src/registration.cpp
  src/tf_utils.cpp
  src/types.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${PCL_DEFINITIONS})

if(EXISTS "${FAST_GICP_LOCAL_LIB_DIR}")
  set_target_properties(${PROJECT_NAME}_core PROPERTIES
    BUILD_RPATH "${FAST_GICP_LOCAL_LIB_DIR}"
    INSTALL_RPATH "${FAST_GICP_LOCAL_LIB_DIR}"
  )
endif()

ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  inha_interfaces
  pcl_conversions
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  fast_gicp
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
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

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME}_core HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  fast_gicp
  geometry_msgs
  inha_interfaces
  pcl_conversions
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

install(EXPORT export_${PROJECT_NAME}_core
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
