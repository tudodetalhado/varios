cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcl-Kinect)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.6.0 REQUIRED)
find_package (OpenCV REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp pclopencvKinect.cpp cloudData.cpp faceDetectionHaar.cpp)
set  (project_HEADERS pclopencvKinect.h cloudData.h faceDetectionHaar.h)
set  (project_FORMS   pclopencvKinect.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

ADD_EXECUTABLE  (pclopencvKinect ${project_SOURCES}
                                 ${project_FORMS_HEADERS}
                                 ${project_HEADERS_MOC})
								 
								 INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})


TARGET_LINK_LIBRARIES (pclopencvKinect ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS})