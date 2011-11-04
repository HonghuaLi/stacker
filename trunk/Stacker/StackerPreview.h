#pragma once

#include "Stacker.h"

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

public slots:
	void setActiveScene(Scene * changedScene);
	void updateActiveObject();
};
