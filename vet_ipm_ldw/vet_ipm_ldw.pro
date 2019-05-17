QT += core
QT -= gui

TARGET = vet_ipm_ldw
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    calibration.cpp \
    ipmtransform.cpp \
    linesegment.cpp \
    cluster.cpp \
    utils.cpp

INCLUDEPATH += D:/Opencv2.4.10/include\
D:/Opencv2.4.10/include/opencv\
D:/Opencv2.4.10/include/opencv2

LIBS += D:\Opencv2.4.10\bin\libopencv_*.dll

HEADERS += \
    global.h \
    calibration.h \
    ipmtransform.h \
    utils.h \
    linesegment.h \
    cluster.h
