# Build instructions for "flightcontrol" package.

cmake_minimum_required(VERSION 3.5)
project(flightcontrol)

# ========================================
# Compiler configuration
# ========================================

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 or newer (20 for compatability with STM32_bridge)
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
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication REQUIRED)
find_package(Protobuf REQUIRED)

# ========================================
# Interface message files
# ========================================

set(msg_files
  "msg/Heartbeat.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# ========================================
# Protobuf generation
# ========================================

# Generate protobuf files
set(PROTO_FILES
  "heartbeat.proto"
)

# Generate C++ protobuf files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Create protobuf library
add_library(leapfrog_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(leapfrog_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(leapfrog_proto ${Protobuf_LIBRARIES})

# ========================================
# Executable: flightmanager
# ========================================

add_executable(flightmanager
  src/flightmanager.cpp
  src/CommandParser.cpp
)

target_include_directories(flightmanager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(flightmanager
  rclcpp
  communication
  Protobuf
)

# Link with communication library and protobuf
target_link_libraries(flightmanager
  communication
  leapfrog_proto
)

rosidl_target_interfaces(flightmanager
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS flightmanager
  DESTINATION lib/${PROJECT_NAME})

# ========================================
# Install launch directory
# ========================================

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ========================================
# Linting / Test Configuration
# ========================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ========================================
# Final package setup
# ========================================

ament_export_include_directories(include)
ament_export_dependencies(communication)

# Remove hardcoded path - use proper CMake target linking instead

ament_package()
