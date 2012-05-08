#pragma once
#include "GL/VBO/VBO.h"
#include "GUI/Viewer/libQGLViewer/QGLViewer/qglviewer.h"
using namespace qglviewer;

class QSegMesh;

class Previewer : public QGLViewer
{
	Q_OBJECT

private:
	QColor backColor;
	QSegMesh * _activeObject;

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
	int stackCount;

	// Active object
	QSegMesh* activeObject();

	// Output
	void saveStackObj( QString fileName, int numStack = 3, double scaleFactor = -1.0);

public slots:
	void setActiveObject(QSegMesh * newObject);
	void updateActiveObject();
	void setStackCount( int num );
};
