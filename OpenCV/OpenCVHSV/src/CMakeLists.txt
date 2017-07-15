cmake_minimum_required(VERSION 2.8)

project(Sample)

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

# Add Executable
add_executable(${PROJECT_NAME} "main.cpp")

# Additional Include Directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Additional Library Directories
link_directories(${OpenCV_LIB_DIR})

# Additional Dependencies
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
