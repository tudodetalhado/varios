TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    CompleteViewer.cpp \
    ImageRenderer.cpp \
    SelectButton.cpp

HEADERS += \
    ColorBasics.h \
    CompleteViewer.h \
    ImageRenderer.h \
    resource.h \
    SelectButton.h \
    stdafx.h
