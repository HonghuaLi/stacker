#pragma once

#include "Scene.h"

enum HVMode { HV_NONE, HV_DEPTH, HV_FACEUNIQUE };

class HiddenViewer : public QGLViewer
{
	Q_OBJECT

private:
	QColor backColor;
	Scene * activeScene;
	HVMode mode;

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
	void setActiveScene(Scene * changedScene);
};
