######################################################################
# @author      : karl (karlsasssie@gmail.com)
# @file        : CMakeLists
# @created     : Friday Apr 07, 2023 15:39:05 CEST
######################################################################

cmake_minimum_required(VERSION 3.15)
project(motherboard CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/canzero/canzero.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/can/can.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/can/socketcan.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/fsm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/init.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/systems.cpp
)


target_include_directories(${PROJECT_NAME} 
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/canzero
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/util
)
