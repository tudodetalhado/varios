cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcdio_test)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} "-std=c++11")

add_executable (pcdio_test pcdio_test.cpp)
target_link_libraries (pcdio_test ${PCL_LIBRARIES})
