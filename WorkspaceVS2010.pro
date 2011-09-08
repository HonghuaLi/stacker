# ----------------------------------------------------
# This file is generated by the Qt Visual Studio Add-in.
# ------------------------------------------------------

TEMPLATE = app
TARGET = Workspace
DESTDIR = ./Debug
QT += core gui multimedia xml script opengl
CONFIG += debug console
DEFINES += QT_LARGEFILE_SUPPORT QT_MULTIMEDIA_LIB QT_XML_LIB QT_OPENGL_LIB QT_SCRIPT_LIB _CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES QT_DLL
INCLUDEPATH += ./GeneratedFiles \
    ./GeneratedFiles/$(Configuration) \
    . \
    ./libQGLViewer \
    ./OpenMesh/src \
    ./OpenNL \
    ./Eigen \
    ./GUI \
    ./GraphicsLibrary \
    ./Spline \
    ./Utility \
    ./GraphicsLibrary/SurfaceMesh \
    ./Stacker
LIBS += -L"./libQGLViewer/QGLViewer/lib" \
    -L"./GL" \
    -L"./OpenMesh/lib" \
    -lGLee \
    -lQGLViewerd2 \
    -lopengl32 \
    -lglu32
PRECOMPILED_HEADER = StdAfx.h
DEPENDPATH += .
MOC_DIR += ./GeneratedFiles/debug
OBJECTS_DIR += debug
UI_DIR += ./GeneratedFiles
RCC_DIR += ./GeneratedFiles
include(Workspace.pri)
win32:RC_FILE = Workspace.rc
