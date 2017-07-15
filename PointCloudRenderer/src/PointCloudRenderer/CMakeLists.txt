#######################################################################
# Setting Target_Name as current folder name
get_filename_component(TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project( ${TARGET_NAME} )



set  (SOURCE_FILES		PCLMesh.cpp 
						CubeMesh.cpp 
						AboutDialog.cpp 
						GLWidget.cpp 
						MainWindow.cpp 
						AppMain.cpp )

set  (HEADER_FILES		PCLMesh.h 
						CubeMesh.h 
						AboutDialog.h 
						GLWidget.h 
						MainWindow.h)

set  (FORM_FILES		MainWindow.ui 
						AboutDialog.ui )


qt5_wrap_ui( HEADER_UI_FILES ${FORM_FILES} )



source_group ("Form Files" FILES ${FORM_FILES})
source_group ("Form Header Files" FILES ${HEADER_UI_FILES})

include_directories (${CMAKE_SOURCE_DIR})


add_executable(
  ${TARGET_NAME}
  ${SOURCE_FILES}
  ${HEADER_FILES}
  ${HEADER_UI_FILES}
)

target_link_libraries (	
	${TARGET_NAME} 
	Qt5::OpenGL
	Qt5::Widgets
	${PCL_LIBRARIES}
	${OPENGL_LIBRARIES}
)


#######################################################################
#################### SETUP OUTPUT DIRECTORY ###########################
set( CMAKE_INSTALL_PREFIX ${PROJECT_ROOT_DIR}/lib_install )
# First for the generic no-config case (e.g. with mingw)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR} )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR} )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR} )
# Second, for multi-config builds (e.g. msvc)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    #STRING( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${OUTPUTCONFIG}_${PROJECT_PLATFORM} )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${OUTPUTCONFIG}_${PROJECT_PLATFORM} )
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${OUTPUTCONFIG}_${PROJECT_PLATFORM} )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )
#######################################################################
