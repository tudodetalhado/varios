cmake_minimum_required(VERSION 3.6)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

# Create Project
project(pclopencvKinect)

# Set StartUp Project
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "pclopencvKinect")

# Find Kinect v.2
find_package(KinectSDK2 REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find PCL Package
set(PCL_ROOT $ENV{PCL_ROOT})
find_package(PCL 1.8.0 REQUIRED)

# Find VTK Package
#set(VTK_DIR "$ENV{PCL_ROOT}/3rdParty/VTK")
find_package(VTK 7.1 REQUIRED)

# Find Qt Package
set(QTDIR $ENV{QTDIR})
set(CMAKE_PREFIX_PATH ${QTDIR})
find_package(Qt5Widgets REQUIRED)

# Set Automatic MOC
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)

set(project_SOURCES main.cpp pclopencvKinect.cpp cloudData.cpp faceDetectionHaar.cpp kinect2_grabber.cpp)
set(project_HEADERS pclopencvKinect.h cloudData.h faceDetectionHaar.h kinect2_grabber.h)
set(project_FORMS   pclopencvKinect.ui)
#set (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

# Add Executable
add_executable(pclopencvKinect ${project_SOURCES}
                               ${project_FORMS_HEADERS}
                               ${project_HEADERS_MOC})

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
target_link_libraries(pclopencvKinect Qt5::Widgets)
target_link_libraries(pclopencvKinect ${PCL_LIBRARIES})
target_link_libraries(pclopencvKinect ${VTK_LIBRARIES})
target_link_libraries(pclopencvKinect ${KinectSDK2_LIBRARIES})
target_link_libraries(pclopencvKinect ${OpenCV_LIBS})