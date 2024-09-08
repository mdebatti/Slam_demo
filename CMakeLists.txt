cmake_minimum_required(VERSION 3.10)

# For filesystem operations
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set the project name and version
project(SLAM VERSION 1.0.0)

# Include directories
include_directories(thirdparty/eigen)

# Add the executable
add_executable(SLAM_executable
    main.cpp
    KalmanFilterStrategy.cpp
    KalmanFilterStrategy.h
    KalmanFilter.h
    KalmanFilter.cpp
    constants.h
    FileIO.h
    FileIO.cpp
    SimulationSetup.h
    SimulationSetup.cpp
    functions.h
    functions.cpp
    types.h
    Logger.h
    Logger.cpp
)

# Add any necessary libraries
# target_link_libraries(SLAM_executable <list of libraries>)
