TEMPLATE = app
TARGET = Workspace
DESTDIR = ./
QT += core gui xml opengl
CONFIG += debug console
DEFINES += QT_LARGEFILE_SUPPORT QT_XML_LIB QT_OPENGL_LIB qh_QHpointer QT_DLL

INCLUDEPATH += ./GeneratedFiles \
    ./GeneratedFiles/Debug \
    . \
    ./GraphicsLibrary/Mesh/SurfaceMesh \
    ./Utility \
    ./Stacker \
    ./GraphicsLibrary/Skeleton \
    ./GraphicsLibrary/Skeleton/Solver/UmfPack_include/UMFPACK \
    ./GraphicsLibrary/Skeleton/Solver/UmfPack_include/AMD \
    ./GraphicsLibrary/Skeleton/Solver/UmfPack_include/UFconfig \

win32{
    LIBS += -L"./GUI/Viewer/libQGLViewer/QGLViewer/lib" \
        -L"./GraphicsLibrary/Skeleton/Solver/lib" \
        -lopengl32 \
        -lglu32 \
        -lQGLViewerd2 \
        -llibamd \
        -llibumfpack
}

unix {
    LIBS += -L$$PWD/GraphicsLibrary/Skeleton/Solver/lib/ -lumfpack
    LIBS += -L$$PWD/GraphicsLibrary/Skeleton/Solver/lib/ -lamd

    LIBS += -lGLEW -lGLU -lGL -lQGLViewer
}

DEPENDPATH += .
MOC_DIR += ./GeneratedFiles/debug
OBJECTS_DIR += debug
UI_DIR += ./GeneratedFiles
RCC_DIR += ./GeneratedFiles
include(Workspace.pri)
