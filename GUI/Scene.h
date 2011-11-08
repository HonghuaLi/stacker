#pragma once

#include <QColorDialog>
#include <QQueue>
#include <QKeyEvent>

#include "Macros.h"

#include "QGLViewer/qglviewer.h"
using namespace qglviewer;

#include "QSegMesh.h"
#include "VBO.h"
#include "Wire.h"
#include "QFFD.h"

class Offset;

enum ViewMode { VIEW, SELECTION, MODIFY };
enum SelectMode { NONE, MESH, SKELETON_NODE, SKELETON_EDGE, SKELETON_FACES, RECONSTRUCTED_POINTS, VERTEX};
enum ModifyMode { DEFAULT, CP_REF_VECTOR, MOVE_VERTEX };


class Scene : public QGLViewer{

	Q_OBJECT

public:
	Scene(QString loadObject = "", QWidget *parent = 0);

	// Setup scene
	virtual void init();
	void setupCamera();
	void setupLights();

	// OpenGL Drawing and Buffer
	virtual void draw();
	virtual void drawWithNames();
	virtual void postDraw();

	// VBO
	QMap<QString, VBO> vboCollection;
	void updateVBOs();

	// Scene Visualizations
	void drawCornerAxis();

	// Mouse & Keyboard stuff
	virtual void mousePressEvent(QMouseEvent* e);
	virtual void mouseReleaseEvent(QMouseEvent* e);
	virtual void mouseMoveEvent(QMouseEvent* e);
	virtual void keyPressEvent(QKeyEvent *e);

	// Focus stuff
	virtual void focusInEvent(QFocusEvent * event);
	virtual void focusOutEvent(QFocusEvent * event);

	// SELECTION
	virtual void postSelection(const QPoint& point);

	// STATE
	ViewMode viewMode;
	SelectMode selectMode;
	ModifyMode modifyMode;

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
	ManipulatedFrame * activeFrame;
	QVector<Wire> activeWires;
	QFFD * activeDeformer;

public:
	QSegMesh * activeObject();
	QString activeObjectId;
	bool isEmpty();

public slots:
	void insertObject( QString fileName );
	void setActiveWires( QVector<Wire> );
	void setActiveDeformer( QFFD * );
	void updateActiveObject();
	void updateSegment(QString objId);

signals:
	void focusChanged( Scene* );
	void objectInserted(  );
	void newSceneCreated();
	void objectModified();
};
