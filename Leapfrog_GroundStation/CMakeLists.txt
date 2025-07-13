# Build instructions for Ground Station packages

cmake_minimum_required(VERSION 3.8)
project(leapfrog_groundstation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flightcontrol REQUIRED)

# Include directories
include_directories(
  src/ground_station/include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${flightcontrol_INCLUDE_DIRS}
)

# Add ground station executable
add_executable(ground_station
  # Main source files
  src/ground_station/src/GroundStation.cpp
  src/ground_station/src/GroundStation_Terminal.cpp
  
  # Properties source files
  src/ground_station/src/properties/BaseProperties.cpp
  
  # Communication source files
  src/ground_station/src/communications/Serial.cpp
  src/ground_station/src/communications/Communication.cpp
  
  # Test/Utility files
  src/Echo.cpp
  src/Echo2.cpp
)

# Link against dependencies
ament_target_dependencies(ground_station
  rclcpp
  std_msgs
  flightcontrol
)

# Install targets
install(TARGETS
  ground_station
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
