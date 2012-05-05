#pragma once

#include "GUI/Scene.h"
#include "Offset.h"

class StackerPanel;

class Previewer : public QGLViewer
{
	Q_OBJECT

private:
	QColor backColor;
	Scene * activeScene;

public:
	Previewer(QWidget * parent = 0);

	// Setup scene
	void init();
	void setupCamera();
	void setupLights();

	void preDraw();
	void draw();
	void postDraw();

	// VBO
	QMap<QString, VBO> vboCollection;
	void updateVBOs();
	void setRenderMode( RENDER_MODE toMode );

	// Keyboard
	virtual void keyPressEvent(QKeyEvent *e);

	// Stacking parameters
	Vec3d stackDirection;
	int stackCount;

	// Active object
	QSegMesh* activeObject();

	// Output
	void saveStackObj( QString fileName, int numStack = 3, double scaleFactor = -1.0);

public slots:
	void setActiveScene(Scene * toScene);
	void updateActiveObject();
	void setStackCount( int num );
};
