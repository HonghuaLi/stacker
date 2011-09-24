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

public slots:
	void setActiveScene(Scene * changedScene);
};
