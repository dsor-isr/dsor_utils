cmake_minimum_required(VERSION 3.1)
project(dsor_utils)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
)

# Make sure the system has eigen3 installed 
find_package(Eigen3 REQUIRED)

# Create the include dependencies for this library
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

# Define the compile options for the library
add_compile_options(-std=c++17 -Wall -O3) 

# Declare a cpp library
add_library(${PROJECT_NAME}
  src/spherical_coordinates.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Install the compiled files in the standard ROS location
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install the header file so that the API is available to other packages
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)