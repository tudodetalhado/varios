cmake_minimum_required( VERSION 3.6 )

# Create Project
project( FirstMorris )
add_executable( FirstMorris main.cpp )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "FirstMorris" )

# Find Package
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

set( OpenCV_DIR "C:/OpenCV/build" )
option( OpenCV_STATIC OFF )
find_package( OpenCV REQUIRED )

# Required Viz Module
if( OpenCV_FOUND )
  if( NOT "opencv_viz" IN_LIST OpenCV_LIBS )
    message( FATAL_ERROR "not found opencv_viz module." )
  endif()
endif()

# Set Static Link Runtime Library
if( OpenCV_STATIC )
  foreach( flag_var
           CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
           CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
           CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
           CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO )
    if( ${flag_var} MATCHES "/MD" )
      string( REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}" )
    endif()
  endforeach()
endif()

if(OpenCV_FOUND )
  # Additional Include Directories
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  # Additional Library Directories
  link_directories( ${OpenCV_LIB_DIR} )

  # Additional Dependencies
  target_link_libraries( FirstMorris ${OpenCV_LIBS} )
endif()
