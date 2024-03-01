cmake_minimum_required(VERSION 3.8)
project(pc_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(raylib 3.0 REQUIRED)

ament_auto_add_executable(controller src/controller.cpp)
target_link_libraries(controller raylib)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_auto_add_executable(auto_controller src/auto_controller.cpp)
target_link_libraries(auto_controller raylib)
target_include_directories(auto_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(auto_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../install)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
