cmake_minimum_required(VERSION 3.8)
project(laser_open_vins_core)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  launch
  config
  rviz
  plotjuggler
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY scripts tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
