#-------------------------------------------------
#
# Project created by QtCreator 2017-03-26T10:27:50
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QKinectPlayer
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    MainWindow.cpp \
    Main.cpp \
    GLDepthBufferRenderer.cpp \
    GLWidget.cpp \
    QImageWidget.cpp \
    QKinectPlayerCtrl.cpp \
    QOpenGLTrackballWidget.cpp \
    KinectFrame.cpp \
    QKinectFile.cpp \
    QKinectFrame.cpp \
    QKinectGrabber.cpp \
    QKinectGrabberFromFile.cpp \
    QKinectIO.cpp

HEADERS  += \
    MainWindow.h \
    GLDepthBufferRenderer.h \
    GLWidget.h \
    QImageWidget.h \
    QKinectPlayerCtrl.h \
    QOpenGLTrackballWidget.h \
    KinectFrame.h \
    QKinectFile.h \
    QKinectFrame.h \
    QKinectGrabber.h \
    QKinectGrabberFromFile.h \
    QKinectIO.h

FORMS    += \
    MainWindow.ui

RESOURCES += \
    QKinectPlayer.qrc

DISTFILES += \
    CMakeLists.txt
