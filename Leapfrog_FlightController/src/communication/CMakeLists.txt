# Build instructions for "communication" package.

cmake_minimum_required(VERSION 3.5)
project(communication)

# ========================================
# Compiler configuration
# ========================================
# Default to C++14 or newer (20 to compatability with STM32_bridge)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# ========================================
# Dependencies
# ========================================
# Required ROS 2 build tools
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ========================================
# Include directories
# ========================================

include_directories(include)

# ========================================
# Build the shared library
# ========================================

add_library(communication SHARED
  src/Serial.cpp
  src/Communication.cpp
)

target_include_directories(communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>       # for local build context
  $<INSTALL_INTERFACE:include>                                 # for installed package context
)

ament_target_dependencies(communication
  rclcpp
)

# ========================================
# Install directives
# ========================================

# Install public headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install the built library (shared object)
install(TARGETS communication
  EXPORT export_communication                                    # Ensures it's visible to downstream packages
  ARCHIVE DESTINATION lib                                        # .a static libraries
  LIBRARY DESTINATION lib                                        # .so shared libraries
  RUNTIME DESTINATION bin                                        # executable stubs (if any)
)

# ========================================
# Final package setup
# ========================================

ament_export_include_directories(include)
ament_export_libraries(communication)
ament_export_targets(export_communication)                      # Registers the target

ament_package()
