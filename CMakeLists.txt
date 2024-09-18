cmake_minimum_required(VERSION 3.8)
project(dual_arm_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config DESTINATION share/${PROJECT_NAME}/)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/trajectory_loader.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
