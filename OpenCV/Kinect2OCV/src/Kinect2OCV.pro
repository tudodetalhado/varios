TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    KinectFaceMat.cpp \
    KinectSource.cpp \
    KinectStreamsMat.cpp

HEADERS += \
    KinectFaceMat.hpp \
    KinectSource.hpp \
    KinectStreamsMat.hpp \
    helper.hpp

DISTFILES += \
    CMakeLists.txt
