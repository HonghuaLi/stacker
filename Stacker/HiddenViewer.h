#pragma once

#include "GUI/Viewer/libQGLViewer/QGLViewer/qglviewer.h"
using namespace qglviewer;

#include "GraphicsLibrary/Mesh/SurfaceMesh/Vector.h"

enum HVMode { HV_NONE, HV_DEPTH, HV_FACEUNIQUE };

class QSegMesh;

struct ObjectTranformation{
	Vec t;
	Quaternion rot;
	Vec3d bbmin;
	Vec3d bbmax;
};

class HiddenViewer : public QGLViewer
{
	Q_OBJECT

private:
	QColor backColor;
	QSegMesh * _activeObject;
	HVMode mode;
	int size;

public:
	HiddenViewer(QWidget * parent = 0);

	void init();
	void setupCamera();
	void setupLights();
	
	void preDraw();
	void draw();

	void setMode(HVMode toMode);
	QSegMesh* activeObject();

	void* readBuffer( GLenum format, GLenum type );

	ObjectTranformation objectTransformation;

public slots:
	void setActiveObject(QSegMesh * changedObject);
	void setResolution( int newRes );

};
