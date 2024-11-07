cmake_minimum_required(VERSION 3.8)
project(hc10dtp_gzsim)

# Check if the compiler being used is GNU's C++ compiler (g++) or Clang.
# Add compiler flags for all targets that will be defined later in the 
# CMakeLists file. These flags enable extra warnings to help catch
# potential issues in the code.
# Add options to the compilation process
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate and configure packages required by the project.
find_package(controller_manager REQUIRED)

find_package(xacro REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Define a CMake variable named dependencies that lists all
# ROS 2 packages and other dependencies the project requires.
set(dependencies
  gz_ros2_control
)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY config worlds
  DESTINATION share/${PROJECT_NAME}
)
ament_package()