QT += core
QT -= gui
QT += serialport

CONFIG += c++11

TARGET = quacon
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += quacon/quacon.cpp

win32:contains(QMAKE_HOST.arch, x86_64):{
    INCLUDEPATH += "c:/opencv31/build/include"
    LIBS += c:/opencv31/build/x64/vc14/lib/opencv_world310.lib
}
