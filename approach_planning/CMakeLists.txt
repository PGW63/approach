cmake_minimum_required(VERSION 3.8)
project(approach_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(approach_mapping REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(approach_planning_lib
  src/config_io.cpp
  src/planner.cpp
)

target_include_directories(approach_planning_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(approach_planning_lib PUBLIC
  approach_mapping::approach_map_lib
  yaml-cpp
)

install(TARGETS
  approach_planning_lib
  EXPORT export_approach_planning_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(approach_mapping yaml-cpp)
ament_export_targets(export_approach_planning_lib HAS_LIBRARY_TARGET)

install(EXPORT export_approach_planning_lib
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
