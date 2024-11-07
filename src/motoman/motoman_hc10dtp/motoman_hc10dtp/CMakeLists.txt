cmake_minimum_required(VERSION 3.8)
project(motoman_hc10dtp)

find_package(ament_cmake REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()