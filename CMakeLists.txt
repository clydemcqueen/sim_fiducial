cmake_minimum_required(VERSION 3.5)
project(sim_fiducial)

# Find packages
find_package(ament_cmake REQUIRED)

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
