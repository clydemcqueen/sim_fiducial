cmake_minimum_required(VERSION 3.5)
project(sim_fiducial)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon
if ($ENV{CLION_IDE})
  message("Running inside CLion")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/tello_msgs/share/tello_msgs/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()


# Find packages
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)  # Note uppercase variables in /usr/lib/x86_64-linux-gnu/cmake/gazebo/gazebo-config.cmake
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Local includes
include_directories(
  include
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tello_msgs_INCLUDE_DIRS}
  )

# Motion plugin moves the camera around in various patterns
add_library(MotionPlugin SHARED
  src/motion_plugin.cpp
  )
ament_target_dependencies(MotionPlugin
  gazebo_dev
  )

add_library(ElementalCameraPlugin SHARED
  src/elemental_camera_plugin.cpp
  )
ament_target_dependencies(ElementalCameraPlugin
  gazebo_dev
  )

ament_export_libraries(
  MotionPlugin
  ElementalCameraPlugin
  )
ament_export_dependencies(gazebo_dev)

## Install plugins
install(TARGETS
  MotionPlugin
  ElementalCameraPlugin
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
