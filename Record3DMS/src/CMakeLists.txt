cmake_minimum_required(VERSION 2.8)

# Working
set(BOOST_ROOT "C:/Boost/Boost1.63/root")

#set(EIGEN_ROOT "C/Eigen/Eigen3.05/root")
#set(EIGEN_INCLUDE_DIRS "C:/Eigen/Eigen3.05/root/include")

set(EIGEN_ROOT "C/Eigen/Eigen3.3/root")
set(EIGEN_INCLUDE_DIRS "C:/Eigen/Eigen3.3/root/include")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

project(Record3DMS)

# Set StartUp Project
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "Record3DMS")

# Find Kinect v.2
find_package(KinectSDK2 REQUIRED)

# Find PCL Package
set(PCL_ROOT $ENV{PCL_ROOT})
find_package(PCL 1.8.0 REQUIRED)

find_package(OpenCV REQUIRED)

# Set Automatic MOC
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Add Executable
add_executable(${PROJECT_NAME} "main.cpp")

# Additional Include Directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${KinectSDK2_INCLUDE_DIRS})

# Preprocessor Definitions
add_definitions(${PCL_DEFINITIONS})

# Additional Library Directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${KinectSDK2_LIBRARY_DIRS})

# Additional Dependencies
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${KinectSDK2_LIBRARIES})
