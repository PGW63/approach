cmake_minimum_required(VERSION 3.8)
project(approach_preprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_library(approach_preprocess_lib
  src/preprocess.cpp
)

target_include_directories(approach_preprocess_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(approach_preprocess_lib
  ${PCL_LIBRARIES}
)

target_compile_definitions(approach_preprocess_lib PUBLIC ${PCL_DEFINITIONS})

install(TARGETS
  approach_preprocess_lib
  EXPORT export_approach_preprocess_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(PCL)
ament_export_targets(export_approach_preprocess_lib HAS_LIBRARY_TARGET)

install(EXPORT export_approach_preprocess_lib
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
