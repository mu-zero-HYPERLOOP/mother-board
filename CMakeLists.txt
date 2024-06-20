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
  ${CMAKE_CURRENT_SOURCE_DIR}/src/control/velocity.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/acceleration.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/arming45.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/calibrating.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/cruising.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/deceleration.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/disarming45.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/guidance_stable.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/idle.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/init.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/levitation_stable.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/precharge.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/ready.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/restarting.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/shutdown.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/start_guidance.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/start_levitation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/stop_guidance.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/states/stop_levitation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/error_handling.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/fsm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm/sdc.cpp

)

target_include_directories(${PROJECT_NAME} 
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE src/canzero
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/fsm
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/util
)
