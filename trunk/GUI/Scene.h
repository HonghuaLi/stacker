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
class QManualDeformer;

enum ViewMode { VIEW, SELECTION, MODIFY };
enum SelectMode { SELECT_NONE, MESH, SKELETON_NODE, SKELETON_EDGE, 
	SKELETON_FACES, RECONSTRUCTED_POINTS, VERTEX, 
	CONTROLLER, CONTROLLER_ELEMENT, FFD_DEFORMER,VOXEL_DEFORMER};

class Scene : public QGLViewer{

	Q_OBJECT

public:
	Scene(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WFlags flags = 0);

	// OpenGL
	ViewMode viewMode;
	QColor backColor;
	void init();
	void setupCamera();
	void setupLights();
	void draw();
	void drawWithNames();
	void postDraw();
	void drawObject();
	void drawStacking();
	void drawGroups();
	void drawCornerAxis();
	void setViewMode(ViewMode toMode);

	// VBO
	QMap<QString, VBO> vboCollection;
	void updateVBOs();
	void setRenderMode(RENDER_MODE toMode);

	// Mouse & Keyboard stuff
	void mousePressEvent(QMouseEvent* e);
	void mouseReleaseEvent(QMouseEvent* e);
	void mouseMoveEvent(QMouseEvent* e);
	void wheelEvent(QWheelEvent* e);
	void keyPressEvent(QKeyEvent *e);

	// Focus, close
	void focusInEvent(QFocusEvent * event);
	void focusOutEvent(QFocusEvent * event);
	void closeEvent( QCloseEvent * event );
	void resizeEvent ( QResizeEvent * event );

	// SELECTION
	SelectMode selectMode;
	QVector<int> selection;
	void setSelectMode(SelectMode toMode);	
	void postSelection(const QPoint& point);

	// Stacker stuff
	StackerPanel * sp;
	bool isShowStacked;
	bool isDrawOffset;

	// Deformer
	ManipulatedFrame * activeFrame;
	QFFD * activeDeformer;
	VoxelDeformer * activeVoxelDeformer;
	QManualDeformer * defCtrl;

	// Sub scenes
	QVector<SubScene*> subScenes;
	void addSubScene(int scene_width);
	void resizeSubScenes();

	// TEXT ON SCREEN
	QQueue<QString> osdMessages;
	QTimer *timer;

	// Objects in the scene
	QSegMesh * activeMesh;
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
	void print(QString message, long age = 1000);
	void dequeueLastMessage();

signals:
	void gotFocus( Scene* );
	void lostFocus( Scene* );
	void objectInserted();
	void exportActiveObject( QSegMesh* newMesh );
	void sceneClosed( Scene* );
	void objectDiscarded( QString );
	void groupsChanged();
};
