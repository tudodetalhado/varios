TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../libs/opencv/ -lopencv_world320
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../libs/opencv/ -lopencv_world320d

INCLUDEPATH += $$PWD/../../libs/opencv
DEPENDPATH += $$PWD/../../libs/opencv
