#-------------------------------------------------
#
# Project created by QtCreator 2013-09-27T15:43:43
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ConProd2
TEMPLATE = app


SOURCES += main.cpp\
        conproddialog.cpp \
    consumer.cpp \
    producer.cpp

HEADERS  += conproddialog.h \
    consumer.h \
    producer.h \
    common.h \
    myConstants.h

FORMS    += conproddialog.ui
