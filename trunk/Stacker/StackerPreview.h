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
	Offset * activeOffset;


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
	void setActiveOffset(Offset * offset);

public slots:
	void setActiveScene(Scene * toScene);
	void updateActiveObject();
};
