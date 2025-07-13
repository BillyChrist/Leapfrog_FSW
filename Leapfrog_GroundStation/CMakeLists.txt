# Build instructions for Ground Station packages

cmake_minimum_required(VERSION 3.8)
project(leapfrog_groundstation)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Protobuf
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

# Include directories
include_directories(
  src/ground_station/include
)

# Add ground station executable
add_executable(ground_station
  src/ground_station/src/GroundStation.cpp
  src/ground_station/src/GroundStation_Terminal.cpp
  src/Echo.cpp
  src/ground_station/src/communications/Serial.cpp
  src/ground_station/src/communications/Communication.cpp
  src/ground_station/src/heartbeat.pb.cc
)

target_link_libraries(ground_station ${Protobuf_LIBRARIES})

# Install targets
install(TARGETS
  ground_station
  DESTINATION bin
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
