cmake_minimum_required(VERSION 3.6)

# Working
set(BOOST_ROOT "C:/Boost/Boost1.63/root")
set(EIGEN_ROOT "C/Eigen/Eigen3.05/root")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

# Create Project
project(KinectController)

# Set StartUp Project
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "KinectController")

# Find Kinect v.2
find_package(KinectSDK2 REQUIRED)

# Find PCL Package
set(PCL_ROOT $ENV{PCL_ROOT})
find_package(PCL 1.8.0 REQUIRED)

# Find VTK Package
set(VTK_DIR $ENV{VTK_DIR})
find_package(VTK REQUIRED)

# Find Qt Package
set(QTDIR $ENV{QTDIR})
set(CMAKE_PREFIX_PATH ${QTDIR})
find_package(Qt5Widgets REQUIRED)

# Set Automatic MOC
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# Generate MOC Code and UI Header
qt5_wrap_cpp(MOC MainWindow.h)
qt5_wrap_ui(UI_HEADER MainWindow.ui)

# Add Executable
add_executable(KinectController main.cpp MainWindow.h MainWindow.h MainWindow.cpp Kinect.h Kinect.cpp KinectManager.h KinectManager.cpp)

# Additional Include Directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${KinectSDK2_INCLUDE_DIRS})

# Load VTK Settings
include(${VTK_USE_FILE})

# Preprocessor Definitions
add_definitions(${PCL_DEFINITIONS})

# Additional Library Directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${KinectSDK2_LIBRARY_DIRS})

# Additional Dependencies
target_link_libraries(KinectController Qt5::Widgets)
target_link_libraries(KinectController ${PCL_LIBRARIES})
target_link_libraries(KinectController ${VTK_LIBRARIES})
target_link_libraries(KinectController ${KinectSDK2_LIBRARIES})
