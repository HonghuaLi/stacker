#pragma once

#include <QColorDialog>
#include <QQueue>
#include <QKeyEvent>

#include "Utility/Macros.h"

#include "GUI/Viewer/libQGLViewer/QGLViewer/qglviewer.h"
using namespace qglviewer;

#include "GL/VBO/VBO.h"
#include "GraphicsLibrary/Mesh/QSegMesh.h"

class SubScene;

// Stacker stuff
class QFFD;
class VoxelDeformer;
class StackerPanel;
class QDeformController;

enum ViewMode { VIEW, SELECTION, MODIFY };
enum ModifyMode { DEFAULT, CP_REF_VECTOR, MOVE_VERTEX };
enum SelectMode { SELECT_NONE, MESH, SKELETON_NODE, SKELETON_EDGE, 
	SKELETON_FACES, RECONSTRUCTED_POINTS, VERTEX, 
	CONTROLLER, CONTROLLER_ELEMENT, FFD_DEFORMER,VOXEL_DEFORMER};

class Scene : public QGLViewer{

	Q_OBJECT

public:
	Scene(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WFlags flags = 0);

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
	virtual void focusOutEvent(QFocusEvent * event);
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
	void setupSubViewport(int x, int y, int w, int h);
	void endSubViewport();

	// Stacker stuff
	QFFD * activeDeformer;
	VoxelDeformer * activeVoxelDeformer;
	StackerPanel * sp; 	// hack
	bool isShowStacked;
	bool isDrawOffset;
	QDeformController * defCtrl;

	// Sub scenes
	QVector<SubScene*> subScenes;
	void addSubScene(int scene_width);
	void resizeSubScenes();

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
	QSegMesh * activeMesh;

public:
	QSegMesh * activeObject();
	bool isEmpty();

public slots:
	void setActiveObject(QSegMesh* newMesh);
	void updateActiveObject();
	void exportActiveObject();
	void toggleCameraProjection();
	void resetView();
	void setActiveDeformer( QFFD * );
	void setActiveVoxelDeformer( VoxelDeformer * );

signals:
	void gotFocus( Scene* );
	void lostFocus( Scene* );
	void objectInserted();
	void exportActiveObject( QSegMesh* newMesh );
	void sceneClosed( Scene* );
	void objectDiscarded( QString );
	void selectionVector( QVector<int> );

	void groupsChanged();
};
