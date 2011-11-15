#pragma once

#include "Scene.h"
#include "Offset.h"

class StackerPanel;

class StackerPreview : public QGLViewer
{
	Q_OBJECT

private:
	QColor backColor;
	Scene * activeScene;

public:
	StackerPreview(QWidget * parent = 0);

	// Setup scene
	void init();
	void setupCamera();
	void setupLights();

	void preDraw();
	void draw();

	// VBO
	QMap<QString, VBO> vboCollection;
	void updateVBOs();

	// Stacking parameters
	Vec3d stackDirection;

	// Active object
	QSegMesh* activeObject();

public slots:
	void setActiveScene(Scene * toScene);
	void updateActiveObject();
};
