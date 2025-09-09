# Build instructions for "STM32_bridge" package.

cmake_minimum_required(VERSION 3.5)
project(stm32_bridge)

# ========================================
# Compiler configuration
# ========================================

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20 (this project uses structured bindings, etc.)
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
find_package(communication REQUIRED)
find_package(flightcontrol REQUIRED)  # For Heartbeat.msg
find_package(Protobuf REQUIRED)

# ========================================
# Executable definition
# ========================================

add_executable(stm32_bridge
  src/stm32_bridge.cpp
)

target_include_directories(stm32_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_SOURCE_DIR}/src/communication/include
)

ament_target_dependencies(stm32_bridge
  rclcpp
  std_msgs
  flightcontrol
  communication
  Protobuf
)

# Explicitly link the compiled communication library
# target_link_libraries(stm32_bridge
#   ${CMAKE_SOURCE_DIR}/install/communication/lib/libcommunication.so
# )


# ========================================
# Installation
# ========================================

install(TARGETS
  stm32_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
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

ament_package()
