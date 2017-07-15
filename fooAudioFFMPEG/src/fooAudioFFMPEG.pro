# -------------------------------------------------
# Project created by QtCreator 2013-01-28T13:25:05
# -------------------------------------------------
TARGET = fooAudioFFMPEG
TEMPLATE = app
SOURCES += main.cpp \
    mainwindow.cpp
HEADERS += mainwindow.h \
    wrapper.h
FORMS += mainwindow.ui
QMAKE_CXXFLAGS += -D__STDC_CONSTANT_MACROS

LIBS += -pthread
LIBS += -L/usr/local/lib
LIBS += -lavdevice
LIBS += -lavfilter
LIBS += -lpostproc
LIBS += -lavformat
LIBS += -lavcodec
LIBS += -ldl
LIBS += -lXfixes
LIBS += -lXext
LIBS += -lX11
LIBS += -lasound
LIBS += -lSDL
LIBS += -lx264
LIBS += -lvpx
LIBS += -lvorbisenc
LIBS += -lvorbis
LIBS += -ltheoraenc
LIBS += -ltheoradec
LIBS += -logg
LIBS += -lopencore-amrwb
LIBS += -lopencore-amrnb
LIBS += -lmp3lame
LIBS += -lfaac
LIBS += -lz
LIBS += -lrt
LIBS += -lswresample
LIBS += -lswscale
LIBS += -lavutil
LIBS += -lm
