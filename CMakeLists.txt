cmake_minimum_required(VERSION 3.5)
project(sim_fiducial)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

# Motion plugin moves the camera around in various patterns
add_library(MotionPlugin SHARED
  src/motion_plugin.cpp
  )
ament_target_dependencies(MotionPlugin
  gazebo_dev
  )
ament_export_libraries(MotionPlugin)
ament_export_dependencies(gazebo_dev)

## Install plugins
install(TARGETS
  MotionPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# Install directories
install(DIRECTORY
  launch
  models
  sdf
  worlds
  DESTINATION share/${PROJECT_NAME}
  )

# Install Python scripts
install(PROGRAMS
  scripts/inject_entity.py
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
