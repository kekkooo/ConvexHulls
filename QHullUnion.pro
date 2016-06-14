QT += core
QT += gui

CONFIG -= \
    opengl \
    thread \

QMAKE_CXXFLAGS+= -fopenmp
QMAKE_LFLAGS +=  -fopenmp

QMAKE_CXXFLAGS_RELEASE         -= -O2
QMAKE_CXXFLAGS_RELEASE         += -O3 -msse2
QMAKE_CXXFLAGS += -std=c++11


TARGET = QHullUnion
CONFIG += console
CONFIG -= app

TEMPLATE = app

DEFINES += CGAL_HAS_THREADS
DEFINES += CGAL_NDEBUG
DEFINES += CGAL_GMPFR_NO_REFCOUNT
#DEFINES += EIGEN_INTERNAL_DEBUGGING

INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += ../IGL/libigl/include
INCLUDEPATH +=   /usr/include/
LIBS        +=   -L/usr/lib/x86_64-linux-gnu
LIBS        += -L/usr/include/
LIBS        += -lCGAL
LIBS        += -lboost_system
LIBS        += -lboost_thread
#LIBS        += -lboost_thread-mt-s
LIBS        += -lgmp
LIBS        += -lmpfr
#QMAKE_CXXFLAGS += -frounding-math -O3


SOURCES += main.cpp \
    convexhullcreator.cpp \
    segmentation.cpp \
    obb.cpp

HEADERS += \
    convexhullcreator.h \
    polyhedron_converter.h \
    segmentation.h \
    commontypedefs.h \
    obb.h \
    chunion.hpp
