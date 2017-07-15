#-------------------------------------------------
#
# Project created by QtCreator 2013-04-23T11:37:27
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = simpleKinectController
TEMPLATE = app
INCLUDEPATH += "C:/Program Files/Microsoft SDKs/Kinect/v1.7/inc"
INCLUDEPATH += "C:/Program Files/Microsoft SDKs/Windows/v7.1/Include"
LIBS += -L"C:/Program Files/Microsoft SDKs/Kinect/v1.7/lib/amd64/" -lKinect10

SOURCES += main.cpp\
        MainWindow.cpp \
    Kinect.cpp \
    KinectManager.cpp

HEADERS  += MainWindow.h \
    Kinect.h \
    KinectManager.h

FORMS    += MainWindow.ui





