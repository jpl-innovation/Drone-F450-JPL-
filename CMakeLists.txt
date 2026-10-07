cmake_minimum_required(VERSION 3.16)
project(drone_telemetry CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(drone_telemetry src/main.cpp)
target_compile_options(drone_telemetry PRIVATE -Wall -Wextra -Wpedantic)
