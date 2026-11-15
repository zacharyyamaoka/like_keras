# like_keras. Lane RULER wrote this file at M0 with only what kpi/ needs; lane CPP owns the runtime it includes.
cmake_minimum_required(VERSION 3.28)
project(like_keras LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# CPP lane (CONTRACT §1): the C++ runtime, like_keras::rt and lk_component(). CMake never runs Python (§7.4).
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/like_keras.cmake)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/like_keras.cmake)
endif()

# RULER: the frozen measurement targets.
add_subdirectory(kpi)
