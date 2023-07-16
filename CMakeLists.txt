cmake_minimum_required(VERSION 3.26)
project(ControlSystem)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/..)
set (CMAKE_EXPORT_COMPILE_COMMANDS 1)

# plotting libary
find_package(Matplot++ REQUIRED)

# Superior 2D Plotter
include_directories(/opt/homebrew/lib/python3.11/site-packages/numpy/core/include)
find_library(PYTHON_LIB
             NAMES libpython3.11.dylib
             PATHS /opt/homebrew/Cellar/python@3.11/3.11.3/Frameworks/Python.framework/Versions/3.11/lib
             NO_DEFAULT_PATH)

# ODE library 
set(BUILD_ODE_LIB ON CACHE BOOL "Build as a library" FORCE)
add_subdirectory(ODESolver)

# Sensor library
set(BUILD_SENS_LIB ON CACHE BOOL "Build as a library" FORCE)
add_subdirectory(Sensor)

# Sensor library
set(BUILD_EST_LIB ON CACHE BOOL "Build as a library" FORCE)
add_subdirectory(StateEstimator)

add_executable(example.out example.cpp plant.cpp control_system.cpp)
target_link_libraries(example.out Matplot++::matplot)
target_link_libraries(example.out odesolver_lib)
target_link_libraries(example.out sensor_lib)
target_link_libraries(example.out stateestimator_lib)
