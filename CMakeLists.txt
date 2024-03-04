cmake_minimum_required(VERSION 3.5)
project(op3_webots)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(op3_webots
  SHARED
  src/op3_driver.cpp
)

target_compile_definitions(op3_webots PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(webots_ros2_driver op3_driver.xml)

# install(TARGETS op3_driver DESTINATION lib)
target_include_directories(
  op3_webots
  PRIVATE
  include
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  rviz
  resource
  world
  protos
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
