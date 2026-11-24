cmake_minimum_required(VERSION 3.8)
project(gerbil_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds models rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Lets Gazebo resolve model://<name> for everything under models/
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
