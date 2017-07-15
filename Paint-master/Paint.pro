#-------------------------------------------------
#
# Project created by QtCreator 2014-01-06T22:51:05
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Paint
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    document.cpp \
    shape.cpp \
    ellipse.cpp \
    rectangle.cpp \
    scribble.cpp \
    command.cpp \
    floodfill.cpp \
    fill.cpp

HEADERS  += mainwindow.h \
    document.h \
    shape.h \
    command.h \
    floodfill.h

OTHER_FILES += \
    README.md \
    UNLICENSE.txt \
    LICENSE.GPL
