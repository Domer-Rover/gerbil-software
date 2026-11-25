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

# No scripts are installed. Teleop, recording, map saving and the ArUco
# nodes live in the repo's top-level scripts/ and run straight from there,
# e.g. ~/gerbil-software/scripts/teleop_keyboard.sh. No rebuild needed.

ament_package()
