#pragma once

#include "GUI/Viewer/libQGLViewer/QGLViewer/qglviewer.h"
using namespace qglviewer;

enum HVMode { HV_NONE, HV_DEPTH, HV_FACEUNIQUE };

class QSegMesh;

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
	void draw();

	void setMode(HVMode toMode);
	QSegMesh* activeObject();

	void* readBuffer( GLenum format, GLenum type );

public slots:
	void setActiveObject(QSegMesh * changedObject);
	void setResolution( int newRes );
};
