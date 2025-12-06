cmake_minimum_required(VERSION 3.15)
project(piperlib LANGUAGES CXX)

# Set build type based on environment
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)

if (NOT DEFINED ENV{PIPERLIB_CONDA_ENV} OR "$ENV{PIPERLIB_CONDA_ENV}" STREQUAL "")
  set(PIPERLIB_CONDA_ENV "$ENV{HOME}/miniforge3/envs/piperlib")
else()
  set(PIPERLIB_CONDA_ENV $ENV{PIPERLIB_CONDA_ENV})
endif()
message(STATUS "Using conda environment: ${PIPERLIB_CONDA_ENV}")

set(CMAKE_PREFIX_PATH ${PIPERLIB_CONDA_ENV} ${CMAKE_PREFIX_PATH}) # this assumes conda is installed with miniforge3

add_compile_definitions(SDK_ROOT="${CMAKE_BINARY_DIR}/..")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

find_package(pinocchio REQUIRED)
find_package(spdlog REQUIRED)
find_package(Catch2 REQUIRED)
find_package(ruckig REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${PIPERLIB_CONDA_ENV}/include)

# Ensure build-tree RPATHs use $ORIGIN so binaries/modules find colocated libs
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

# Create a single static library that bundles everything
add_library(PiperController SHARED
    src/socket_can.cpp
    src/piper_interface.cpp
    src/utils.cpp
    src/solver.cpp
    src/controller_base.cpp
)
target_link_libraries(PiperController
    spdlog::spdlog_header_only
    pinocchio::pinocchio
    ruckig::ruckig
)

# TESTS

# TESTS (only build if not installing via pip)
if(NOT SKBUILD)
  add_executable(test_interface test/test_interface.cpp)
  target_link_libraries(test_interface PRIVATE PiperController Catch2::Catch2)

  add_executable(test_gravity_compensation test/test_gravity_compensation.cpp)
  target_link_libraries(test_gravity_compensation PRIVATE PiperController)

  add_executable(test_motor_control test/test_motor_control.cpp)
  target_link_libraries(test_motor_control PRIVATE PiperController Catch2::Catch2WithMain)

  add_executable(test_piper_controller test/test_piper_controller.cpp)
  target_link_libraries(test_piper_controller PRIVATE PiperController)

endif()

# EXTRAS
add_executable(resume_emergency_stop extra/resume_emergency_stop.cpp)
target_link_libraries(resume_emergency_stop PRIVATE PiperController)

add_executable(emergency_stop extra/emergency_stop.cpp)
target_link_libraries(emergency_stop PRIVATE PiperController)

# PYTHON
pybind11_add_module(_piperlib src/piper_pybind.cpp)
target_link_libraries(_piperlib PRIVATE PiperController)
set_target_properties(_piperlib PROPERTIES PREFIX "")
set_target_properties(_piperlib PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/piperlib")
set_target_properties(PiperController PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/piperlib")



