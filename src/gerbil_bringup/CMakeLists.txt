cmake_minimum_required(VERSION 3.8)
project(gerbil_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/aruco_detector.py
    scripts/teleop_keyboard.sh
    scripts/teleop_joystick.sh
  DESTINATION lib/${PROJECT_NAME}
)

# teleop_common.sh is sourced by the two teleop scripts, never run directly,
# so it installs as data rather than as a program.
install(
  FILES scripts/teleop_common.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
