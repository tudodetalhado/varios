#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pclopencvKinect_visualizer
TEMPLATE = app

SOURCES += main.cpp\
    pclopencvKinect.cpp \
    kinect2_grabber.cpp \
    cloudData.cpp \
    faceDetectionHaar.cpp

HEADERS  += pclopencvKinect.h \
    kinect2_grabber.h \
    cloudData.h \
    faceDetectionHaar.h

FORMS += pclopencvKinect.ui
