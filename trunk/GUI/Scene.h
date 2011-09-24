#pragma once

#include <QColorDialog>
#include <QQueue>
#include <QKeyEvent>

#include "Macros.h"

#include "QGLViewer/qglviewer.h"
using namespace qglviewer;

#include "QSurfaceMesh.h"
#include "Wire.h"

enum ViewMode { VIEW, SELECTION, MODIFY };
enum SelectMode { NONE, MESH, SKELETON_NODE, SKELETON_EDGE, SKELETON_FACES, RECONSTRUCTED_POINTS, VERTEX};
enum ModifyMode { DEFAULT, CP_REF_VECTOR, MOVE_VERTEX };
enum SpecialRenderMode { REGULAR, DEPTH, UNIQUE_FACES };

class Scene : public QGLViewer{

	Q_OBJECT

public:
	Scene(QString loadObject = "", QWidget *parent = 0);

	// Setup scene
	virtual void init();
	void setupCamera();
	void setupLights();

	// OpenGL Drawing and Buffer
	virtual void preDraw();
	virtual void draw();
	virtual void drawWithNames();
	virtual void postDraw();
	virtual void specialDraw();

	void* readBuffer( GLenum format, GLenum type );

	// Scene Visualizations
	void drawCornerAxis();

	// Mouse & Keyboard stuff
	virtual void mousePressEvent(QMouseEvent* e);
	virtual void mouseReleaseEvent(QMouseEvent* e);
	virtual void mouseMoveEvent(QMouseEvent* e);
	virtual void keyPressEvent(QKeyEvent *e);

	// Focus stuff
	virtual void focusInEvent(QFocusEvent * event);

	// SELECTION
	virtual void postSelection(const QPoint& point);

	// STATE
	ViewMode viewMode;
	SelectMode selectMode;
	ModifyMode modifyMode;
	SpecialRenderMode specialRenderMode;

	QColor backColor;

	void setViewMode(ViewMode toMode);
	void setSelectMode(SelectMode toMode);
	void setModifyMode(ModifyMode toMode);

// TEXT ON SCREEN
public slots:
	void print(QString message, long age = 1000);
	void dequeueLastMessage();

private:
	QQueue<QString> osdMessages;
	QTimer *timer;

// Objects in the scene
private:
	QVector<Wire> activeWires;

public:
	QSurfaceMesh * activeObject();
	QString activeObjectId;

public slots:
	void insertObject( QString fileName );
	uint numObjects();
	void setActiveWires( QVector<Wire> );

signals:
	void focusChanged( Scene* );
	void objectInserted( QSurfaceMesh * );
};
