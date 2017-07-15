cmake_minimum_required( VERSION 3.6 )

# Create Project
project( pcd_video_player )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "pcd_video_player" )

# Find PCL Package
set( PCL_ROOT $ENV{PCL_ROOT} )
find_package( PCL 1.8.0 REQUIRED )

# Find VTK Package
set( VTK_DIR "$ENV{PCL_ROOT}/3rdParty/VTK" )
find_package( VTK REQUIRED )

# Find Qt Package
set( QTDIR $ENV{QTDIR} )
set( CMAKE_PREFIX_PATH ${QTDIR} )
find_package( Qt5Widgets REQUIRED )

# Set Automatic MOC
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )

# Generate MOC Code and UI Header
# qt5_wrap_cpp( MOC pclviewer.h )
qt5_wrap_ui( UI_HEADER pcd_video_player.ui )

# Add Executable
add_executable( pcd_video_player pcd_video_player.cpp pcd_video_player.h pcd_video_player.ui )

# Additional Include Directories
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# Load VTK Settings
include( ${VTK_USE_FILE} )

# Preprocessor Definitions
add_definitions( ${PCL_DEFINITIONS} )

# Additional Library Directories
link_directories( ${PCL_LIBRARY_DIRS} )

# Additional Dependencies
target_link_libraries( pcd_video_player ${PCL_LIBRARIES} ${VTK_LIBRARIES} Qt5::Widgets )
