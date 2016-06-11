QT -= core
QT -= gui

CONFIG -= \
    opengl \
    thread \

#QMAKE_CXXFLAGS += -mmacosx-version-min=10.9
QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.11
QMAKE_CXXFLAGS_RELEASE         -= -O2
QMAKE_CXXFLAGS_RELEASE         += -Oz -msse2
QMAKE_CXXFLAGS                 += -std=c++11


TARGET = QHullUnion
CONFIG += console
CONFIG -= app

TEMPLATE = app
#DEFINES += CGAL_NDEBUG
#DEFINES += EIGEN_INTERNAL_DEBUGGING


INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/boost_1_59_0/BUILD109/include
#INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/cgal-releases-CGAL-4.7/BUILD109/include
#INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/CGAL-4.8/BUILD/include
INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/CGAL-4.8/DEBUG_BUILD/include
INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/mpfr-3.1.3/BUILD109/include
INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/gmp-6.1.0/BUILD109/include
INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/libigl/include
INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/cork/include
INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/eigen
INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/glfw-3.1.2/include
INCLUDEPATH += /Users/francescousai/Documents/Sviluppo/Libs/glfw-3.1.2/src

LIBS    += -L/Users/francescousai/Documents/Sviluppo/Libs/glfw-3.1.2/BUILD/src -lglfw3
LIBS    += -framework Cocoa
LIBS    += -framework OpenGL
LIBS    += -framework IOKit
LIBS    += -framework CoreVideo
LIBS    += -L/Users/francescousai/Documents/Sviluppo/Libs/boost_1_59_0/BUILD109/lib -lboost_system-mt-s
LIBS    += -L/Users/francescousai/Documents/Sviluppo/Libs/boost_1_59_0/BUILD109/lib -lboost_thread-mt-s
#LIBS   += -L/Users/francescousai/Documents/Sviluppo/Libs/cgal-releases-CGAL-4.7/BUILD109/lib -lCGAL
#LIBS   += -L/Users/francescousai/Documents/Sviluppo/Libs/CGAL-4.8/BUILD/lib -lCGAL
LIBS    += -L/Users/francescousai/Documents/Sviluppo/Libs/CGAL-4.8/DEBUG_BUILD/lib -lCGAL
LIBS    += -L/Users/francescousai/Documents/Sviluppo/Libs/mpfr-3.1.3/BUILD109/lib -lmpfr
LIBS    += -L/Users/francescousai/Documents/Sviluppo/Libs/gmp-6.1.0/BUILD109/lib -lgmp
LIBS    += -L/Users/francescousai/Documents/Sviluppo/Libs/cork/lib -lcork



SOURCES += main.cpp \
    convexhullcreator.cpp \
    segmentation.cpp \
    common_paths.cpp \
    iglconvexhullunion.cpp \
    cgal_to_eigen.cpp \
    csg_tree_approach.cpp \
    iglmeshboolean.cpp

HEADERS += \
    convexhullcreator.h \
    polyhedron_converter.h \
    segmentation.h \
    commontypedefs.h \
    common_paths.h \
    iglconvexhullunion.h \
    cgal_to_eigen.h \
    csg_tree_approach.h \
    iglmeshboolean.h
