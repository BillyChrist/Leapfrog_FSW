# Build instructions for Flight Controller packages

cmake_minimum_required(VERSION 3.8)
project(leapfrog_flightcontroller)

# ========================================
# Compiler configuration
# ========================================

# Default to C++20 for all packages
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# Dependencies
# ========================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Protobuf REQUIRED)

# ========================================
# Library path configuration
# ========================================

# Set up library paths
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Add install directory to library path
link_directories("${CMAKE_INSTALL_PREFIX}/lib")

# ========================================
# Include directories
# ========================================

include_directories(
  src/flightcontrol/include
  src/stm32_bridge/include
  src/communication/include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# ========================================
# Protobuf integration
# ========================================

set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/heartbeat.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Make generated files available to subprojects
add_library(leapfrog_proto STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(leapfrog_proto PUBLIC protobuf::libprotobuf)

# ========================================
# Installation
# ========================================

# Install scripts directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(DIRECTORY
    scripts
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ========================================
# Final package setup
# ========================================

ament_package() 