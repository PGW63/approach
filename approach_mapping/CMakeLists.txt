cmake_minimum_required(VERSION 3.8)
project(approach_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(approach_map_lib
  src/map/builder.cpp
  src/map/clearance.cpp
  src/map/config_io.cpp
  src/map/evidence.cpp
  src/map/feasibility.cpp
  src/map/types.cpp
)

target_include_directories(approach_map_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(approach_map_lib PUBLIC yaml-cpp)

install(TARGETS
  approach_map_lib
  EXPORT export_approach_map_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(yaml-cpp)
ament_export_targets(export_approach_map_lib HAS_LIBRARY_TARGET)

install(EXPORT export_approach_map_lib
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
