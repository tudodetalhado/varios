cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

project(sample)
add_executable(sample 
   kinect2_grabber.h main.cpp
   tawara/ebml_element.h
   tawara/file_cluster.h
   tawara/segment.h
   tawara/simple_block.h
   tawara/tawara_impl.h
   tawara/tracks.h
   tawara/track_entry.h
)
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "sample")

# Find Packages
find_package(PCL 1.8 REQUIRED)
find_package(KinectSDK2 REQUIRED)

if(PCL_FOUND AND KinectSDK2_FOUND)
  # Additional Include Directories
  include_directories(${PCL_INCLUDE_DIRS})
  include_directories(${KinectSDK2_INCLUDE_DIRS})
  include_directories(${PROJECT_SOURCE_DIR})

  # Preprocessor Definitions
  add_definitions(${PCL_DEFINITIONS})

  # Additional Library Directories
  link_directories(${PCL_LIBRARY_DIRS})
  link_directories(${KinectSDK2_LIBRARY_DIRS})

  # Additional Dependencies
  target_link_libraries(sample ${PCL_LIBRARIES})
  target_link_libraries(sample ${KinectSDK2_LIBRARIES})
endif()
