
# BELOW IS CMAKE TO MAKE A COMPILED PYTHON EXTENSION MODULE
#cmake_minimum_required(VERSION 3.10)

## Set the project name and version
#project(SLAM VERSION 1.0.0)

## Find Python 3.12 explicitly
#find_package(Python3 3.12 EXACT REQUIRED COMPONENTS Interpreter)

## Use the Python 3.12 interpreter for building the module
#set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})

#add_subdirectory(thirdparty/pybind11)
#include_directories(thirdparty/eigen)

#pybind11_add_module(SLAM_cpp
#    wrappers.cpp
#    kf.h
#    VehicleModel.cpp
#    VehicleModel.h
#    KalmanFilter.h
#    KalmanFilter.cpp
#    constants.h
#    functions.h
#    functions.cpp
#    CMatlabFileReading.h
#    CMatlabFileReading.cpp
#    CDynamicArray.h
#    CDynamicArray.cpp
#    CDoubleVector.h
#    CDoubleVector.cpp
#    main.cpp
#)

cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(SLAM VERSION 1.0.0)

# Include directories
include_directories(thirdparty/eigen)

# Add the executable
add_executable(SLAM_executable
    main.cpp
    wrappers.cpp
    kf.h
    VehicleModel.cpp
    VehicleModel.h
    KalmanFilter.h
    KalmanFilter.cpp
    constants.h
    functions.h
    functions.cpp
    CMatlabFileReading.h
    CMatlabFileReading.cpp
    CDynamicArray.h
    CDynamicArray.cpp
    CDoubleVector.h
    CDoubleVector.cpp
)

# Add any necessary libraries
# target_link_libraries(SLAM_executable <list of libraries>)
