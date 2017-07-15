cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER $ENV{COMPILER})
set(CMAKE_CXX_COMPILER $ENV{COMPILER})

project(PcdBench)

# Find Package
find_package( PCL 1.8 REQUIRED )

# Additional Include Directories
include_directories( ${PCL_INCLUDE_DIRS} )

# Preprocessor Definitions
add_definitions( ${PCL_DEFINITIONS} )

# Additional Library Directories
link_directories( ${PCL_LIBRARY_DIRS} )


add_executable(${PROJECT_NAME} "main.cpp")
