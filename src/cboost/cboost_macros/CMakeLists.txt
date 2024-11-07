cmake_minimum_required(VERSION 3.8)
project(cboost_macros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
install (
  DIRECTORY xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
