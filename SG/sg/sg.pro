TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    sg/vetfrontcarmotionstatejudging_frame_diff.cpp \
    sg/vetstoporgo.cpp \
    sg/vetthiscarmotionstatejudging_frame_diff.cpp \
    sg/vetsg_utils.cpp

HEADERS += \
    sg/vetfrontcarmotionstatejudging_frame_diff.h \
    sg/vetstoporgo.h \
    sg/vetthiscarmotionstatejudging_frame_diff.h \
    sg/vetsg_utils.h

INCLUDEPATH += D:/Opencv2.4.10/include\
D:/Opencv2.4.10/include/opencv\
D:/Opencv2.4.10/include/opencv2\
./sg

LIBS += D:\Opencv2.4.10\bin\libopencv_*.dll
