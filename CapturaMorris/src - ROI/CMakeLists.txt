cmake_minimum_required(VERSION 3.6)

# Working
set(BOOST_ROOT "C:/Boost/Boost1.63/root")
set(EIGEN_ROOT "C/Eigen/Eigen3.05/root")
set(OpenCV_DIR "C:/OpenCV/build")
set(OpenCV_INCLUDE_DIRS "C:/OpenCV/build/include")
set(VTK_DIR "C:/VTK/VTK-build-7.1.1")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

# Create Project
project(CapturaMorris)

# Set StartUp Project
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "CapturaMorris")

# Find Boost
find_package(Boost 1.63.0 COMPONENTS system filesystem REQUIRED)

# Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
link_directories(${OPENGL_LIBRARIES})

# Find Kinect v.2
find_package(KinectSDK2 REQUIRED)

# Find OpenCV
option(OpenCV_STATIC OFF)
find_package(OpenCV REQUIRED)

# Required Viz Module
if(OpenCV_FOUND)
  if(NOT "opencv_viz" IN_LIST OpenCV_LIBS)
    message(FATAL_ERROR "not found opencv_viz module.")
  endif()
endif()

# Set Static Link Runtime Library
if(OpenCV_STATIC)
  foreach(flag_var
           CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
           CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
           CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
           CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
    if(${flag_var} MATCHES "/MD")
      string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
    endif()
  endforeach()
endif()

# Find PCL Package
set(PCL_ROOT $ENV{PCL_ROOT})
find_package(PCL 1.8.0)
#find_package(PCL 1.8.0 REQUIRED)

# Find VTK Package
#set(VTK_DIR $ENV{VTK_DIR})
find_package(VTK REQUIRED)

# Find Qt Package
set(QTDIR $ENV{QTDIR})
set(CMAKE_PREFIX_PATH ${QTDIR})
find_package(Qt5Widgets REQUIRED)


# Set Automatic MOC
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Generate MOC Code and UI Header
qt5_wrap_cpp(MOC MainWindow.h)
qt5_wrap_ui(UI_HEADER MainWindow.ui)

# Add Executable
add_executable(CapturaMorris Main.cpp
    MainWindow.h MainWindow.cpp MainWindow.ui
    KinectWriter.h KinectWriter.cpp
    Utils.h Utils.cpp stdafx.h
    KinectSensor.h KinectSensor.cpp
    KinectReader.h KinectReader.cpp
    ctkRangeSlider.h ctkRangeSlider.cpp)
    #Marker.h Marker.cpp #opencv2/opencv.hpp
    #Filter.h Filter.cpp nanoflann.h) #opencv/cv.h)

# Additional Include Directories
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${KinectSDK2_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Load VTK Settings
include(${VTK_USE_FILE})

# Preprocessor Definitions
add_definitions(${PCL_DEFINITIONS})

# Additional Library Directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${KinectSDK2_LIBRARY_DIRS})
link_directories(${OpenCV_LIB_DIR})

# Additional Dependencies
target_link_libraries(CapturaMorris Qt5::Widgets)
target_link_libraries(CapturaMorris ${Boost_LIBRARIES})
target_link_libraries(CapturaMorris ${PCL_LIBRARIES})
target_link_libraries(CapturaMorris ${VTK_LIBRARIES})
target_link_libraries(CapturaMorris ${OPENGL_LIBRARIES})
target_link_libraries(CapturaMorris ${KinectSDK2_LIBRARIES})
target_link_libraries(CapturaMorris ${OpenCV_LIBS})
