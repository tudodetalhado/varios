cmake_minimum_required (VERSION 3.0 FATAL_ERROR)

# Create folder to allocate cmake owner projects
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#######################################################################
########################### SETUP PROJECT #############################
project(	PointCloudRenderer	)
set (		PROJECT_ROOT_DIR 	"${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "Root directory")
set (		PROJECT_SOURCE_DIR 	"${PROJECT_ROOT_DIR}/src" CACHE INTERNAL "Source directory")
set (		PROJECT_SHADER_DIR 	"${PROJECT_ROOT_DIR}/shaders" CACHE INTERNAL "Shaders directory")
set (		PROJECT_BINARY_DIR 	"${PROJECT_ROOT_DIR}/bin" CACHE INTERNAL "Binary output directory")
#######################################################################


#######################################################################
########################### SETUP PLATFORM ############################
if (WIN32)
	if (CMAKE_CL_64)
		set (PROJECT_PLATFORM "Win64")
	else (CMAKE_CL_64)
		set (PROJECT_PLATFORM "Win32")
	endif (CMAKE_CL_64)
elseif (win32)
	if (CMAKE_CL_64)
		set (PROJECT_PLATFORM "unix64")
	else (CMAKE_CL_64)
		set (PROJECT_PLATFORM "unix32")
	endif (CMAKE_CL_64)
endif (WIN32)
#######################################################################


#######################################################################
################# SETUP LOCAL INCLUDE AND LIBRARIES ###################
include_directories(${PROJECT_ROOT_DIR}/include)
link_directories(	${PROJECT_ROOT_DIR}/lib 
					${PROJECT_ROOT_DIR}/lib/${PROJECT_PLATFORM})
#######################################################################

#set(OPENGL_INCLUDE_DIRS "C:/OpenGL/inc")
#set(OPENGL_LIBRARIES "C:/OpenGL/lib")

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
link_directories(${OPENGL_LIBRARIES}) 

#######################################################################
############################# SETUP QT5 ###############################
find_package (Qt5OpenGL REQUIRED)
find_package (Qt5Widgets REQUIRED)
if (Qt5_POSITION_INDEPENDENT_CODE)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)	# Find includes in corresponding build directories
include_directories	(${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
#######################################################################


#######################################################################
############################# SETUP PCL ###############################
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#######################################################################



add_subdirectory (${PROJECT_SOURCE_DIR}/PointCloudRenderer)





