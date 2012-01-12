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
#include "VoxelDeformer.h"

class Offset;
class StackerPanel;

enum ViewMode { VIEW, SELECTION, MODIFY };
enum ModifyMode { DEFAULT, CP_REF_VECTOR, MOVE_VERTEX };
enum SelectMode { NONE, MESH, SKELETON_NODE, SKELETON_EDGE, 
	SKELETON_FACES, RECONSTRUCTED_POINTS, VERTEX, CONTROLLER, CONTROLLER_ELEMENT};

#include "QDeformController.h"
extern QDeformController * defCtrl;

#include "EditSuggestion.h"

class SubScene;

class Scene : public QGLViewer{

	Q_OBJECT

public:
	Scene(QWidget *parent = 0);

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
	void setRenderMode(RENDER_MODE toMode);

	// Scene Visualizations
	void drawCornerAxis();

	// Mouse & Keyboard stuff
	virtual void mousePressEvent(QMouseEvent* e);
	virtual void mouseReleaseEvent(QMouseEvent* e);
	virtual void mouseMoveEvent(QMouseEvent* e);
	virtual void wheelEvent(QWheelEvent* e);
	virtual void keyPressEvent(QKeyEvent *e);

	// Focus, close
	virtual void focusInEvent(QFocusEvent * event);
	virtual void closeEvent( QCloseEvent * event );

	// SELECTION
	virtual void postSelection(const QPoint& point);
	QVector<int> selection;

	// SHAPING
	void resizeEvent ( QResizeEvent * event );

	// STATE
	ViewMode viewMode;
	SelectMode selectMode;
	ModifyMode modifyMode;

	QColor backColor;

	void setViewMode(ViewMode toMode);
	void setSelectMode(SelectMode toMode);
	void setModifyMode(ModifyMode toMode);

	// draw on screen
	QVector<SubScene*> subScenes;
	void addSubScene(int scene_width);
	void resizeSubScenes();
	void setupSubViewport(int x, int y, int w, int h);
	void endSubViewport();

	// hack
	StackerPanel * sp;

	bool isShowStacked;
	bool isDrawOffset;

// TEXT ON SCREEN
public slots:
	void print(QString message, long age = 1000);
	void dequeueLastMessage();

private:
	QQueue<QString> osdMessages;
	QTimer *timer;


// Objects in the scene
public:
	QVector<EditSuggestion> suggestions;

private:
	ManipulatedFrame * activeFrame;
	QVector<Wire> activeWires;
	QFFD * activeDeformer;
	VoxelDeformer * activeVoxelDeformer;
	QSegMesh * activeMesh;
public:
	QSegMesh * activeObject();
	bool isEmpty();

public slots:
	void setActiveObject(QSegMesh* newMesh);
	void setActiveWires( QVector<Wire> );
	void setActiveDeformer( QFFD * );
	void setActiveVoxelDeformer( VoxelDeformer * );
	void updateActiveObject();
	void exportActiveObject();
	void toggleCameraProjection();

signals:
	void gotFocus( Scene* );
	void objectInserted();
	void exportActiveObject( QSegMesh* newMesh );
	void sceneClosed( Scene* );
	void objectDiscarded( QString );
	void selectionVector( QVector<int> );
	void groupsChanged();

};
