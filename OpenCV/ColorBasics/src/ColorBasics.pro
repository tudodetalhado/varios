TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    ColorBasics.cpp \
    ImageRenderer.cpp

HEADERS += \
    ColorBasics.h \
    ImageRenderer.h \
    resource.h \
    stdafx.h

DISTFILES += \
    CMakeLists.txt
