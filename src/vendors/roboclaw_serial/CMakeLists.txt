cmake_minimum_required(VERSION 3.8)

project(roboclaw_serial LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the header-only library
add_library(roboclaw_serial INTERFACE)

# Provide a namespaced alias target for downstream users
add_library(roboclaw_serial::roboclaw_serial ALIAS roboclaw_serial)

target_include_directories(roboclaw_serial INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the headers
install(DIRECTORY include/roboclaw_serial
    DESTINATION include
)

# Install + export target for downstream `find_package(roboclaw_serial)` usage
install(TARGETS roboclaw_serial
    EXPORT export_${PROJECT_NAME}
    INCLUDES DESTINATION include
)

install(EXPORT export_${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

# Testing
include(CTest) # Provides option for BUILD_TESTING, defaulted to ON
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_roboclaw_serial
        test/test_main.cpp
    )

    target_link_libraries(test_roboclaw_serial
        roboclaw_serial
    )
endif()

ament_package()