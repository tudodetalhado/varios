cmake_minimum_required(VERSION 3.6)

# Working
set(BOOST_ROOT "C:/Boost/Boost1.63/root")
set(EIGEN_ROOT "C/Eigen/Eigen3.05/root")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

# Create Project
project(CoordinateMapper)

# Set StartUp Project
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "CoordinateMapper")

# Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
link_directories(${OPENGL_LIBRARIES})

# Find Kinect v.2
find_package(KinectSDK2 REQUIRED)

# Find OpenCV
set(OpenCV_DIR "C:/OpenCV/build")
option(OpenCV_STATIC OFF)
find_package(OpenCV REQUIRED)

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
set(VTK_DIR $ENV{VTK_DIR})
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
add_executable(CoordinateMapper app.h app.cpp main.cpp util.h)

# Additional Include Directories
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
target_link_libraries(CoordinateMapper Qt5::Widgets)
target_link_libraries(CoordinateMapper ${PCL_LIBRARIES})
target_link_libraries(CoordinateMapper ${VTK_LIBRARIES})
target_link_libraries(CoordinateMapper ${OPENGL_LIBRARIES})
target_link_libraries(CoordinateMapper ${KinectSDK2_LIBRARIES})
target_link_libraries(CoordinateMapper ${OpenCV_LIBS})
