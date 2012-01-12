#include "Workspace.h"
#include <QVBoxLayout>
#include <QFileInfo>

Workspace::Workspace(QWidget *parent, Qt::WFlags flags)	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	ui.leftDockWidget->setLayout(new QVBoxLayout);
	ui.rightDockWidget->setLayout(new QVBoxLayout);

	sp = new StackerPanel();
	ui.leftDockWidget->layout()->addWidget(sp);

	wp = new WiresPanel();
	ui.rightDockWidget->layout()->addWidget(wp);

	dp = new DeformerPanel();
	ui.rightDockWidget->layout()->addWidget(dp);

	vdp = new QVoxelDeformerPanel();
	ui.rightDockWidget->layout()->addWidget(vdp);

	tp = new TransformationPanel();
	ui.rightDockWidget->layout()->addWidget(tp);

	gp = new GroupPanel();
	ui.rightDockWidget->layout()->addWidget(gp);

	cp = new ControllerPanel();
	ui.rightDockWidget->layout()->addWidget(cp);

	// Create MeshDoc, where stores all the meshes
	mDoc = new QMeshDoc();
	connect(ui.actionImportObject, SIGNAL(triggered()), mDoc, SLOT(importObject()));

	// Add new scene action
	connect(ui.actionNewScene, SIGNAL(triggered()), SLOT(addNewScene()));

	// Create new scene when we start by default
	sceneCount = 0;
	addNewScene();
}

Workspace::~Workspace()
{
	
}

void Workspace::addNewScene()
{
	Scene * newScene = new Scene;

	ui.sceneArea->addSubWindow(newScene);
	sceneCount++;

	newScene->showMaximized();
	newScene->setWindowTitle("Untitled");

	// Workspace window
	connect(newScene, SIGNAL(gotFocus(Scene*)), SLOT(setActiveScene(Scene*)));

	// MeshDoc
	connect(newScene, SIGNAL(objectDiscarded(QString)), mDoc, SLOT(deleteObject(QString)));
	
	// Stack panel
	connect(newScene, SIGNAL(gotFocus(Scene*)), sp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(objectInserted()), sp, SLOT(updateActiveObject()));
	connect(newScene, SIGNAL(sceneClosed(Scene*)), sp, SLOT(setActiveScene(Scene*)));
	connect(sp, SIGNAL(printMessage(QString)), newScene, SLOT(print(QString)));
	connect(sp, SIGNAL(objectModified()), newScene, SLOT(updateActiveObject()));

	// Wires
	connect(newScene, SIGNAL(gotFocus(Scene*)), wp, SLOT(setActiveScene(Scene*)));
	connect(wp, SIGNAL(wiresFound(QVector<Wire>)), newScene, SLOT(setActiveWires(QVector<Wire>)));
	connect(wp, SIGNAL(wiresFound(QVector<Wire>)), newScene, SLOT(updateGL()));

	// Deformation
	connect(newScene, SIGNAL(gotFocus(Scene*)), dp, SLOT(setActiveScene(Scene*)));
	connect(dp, SIGNAL(deformerCreated(QFFD *)), newScene, SLOT(setActiveDeformer(QFFD *)));
	
	connect(newScene, SIGNAL(gotFocus(Scene*)), vdp, SLOT(setActiveScene(Scene*)));
	connect(vdp, SIGNAL(deformerCreated(VoxelDeformer *)), newScene, SLOT(setActiveVoxelDeformer(VoxelDeformer *)));

	// Object transformation
	connect(newScene, SIGNAL(gotFocus(Scene*)), tp, SLOT(setActiveScene(Scene*)));
	connect(tp, SIGNAL(objectModified()), newScene, SLOT(updateActiveObject()));
	connect(tp, SIGNAL(objectModified()), sp, SLOT(updateActiveObject()));

	// Groups
	connect(newScene, SIGNAL(gotFocus(Scene*)), gp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(groupsChanged()), gp, SLOT(updateWidget()));

	// View operations
	connect(ui.actionCameraProjection, SIGNAL(triggered()), newScene, SLOT(toggleCameraProjection()));

	// Explicit updates
	sp->setActiveScene(newScene);
	gp->setActiveScene(newScene);
	cp->setActiveScene(newScene);

	this->setActiveScene(newScene);

	newScene->sp = sp;
}

void Workspace::setActiveScene(Scene* scene)
{
	activeScene = scene;

	QString title = QString("%1 - %2")
		.arg(QFileInfo(QApplication::applicationFilePath()).baseName())
		.arg(scene->windowTitle());

	this->setWindowTitle(title);

	// Disconnect Mesh Doc with from all scenes
	foreach (QMdiSubWindow *window, ui.sceneArea->subWindowList()) {
		Scene *s = qobject_cast<Scene *>(window->widget());
		s->disconnect(mDoc);
		s->disconnect(ui.actionExportObject);
	}
	
	activeScene->connect(mDoc, SIGNAL(objectImported(QSegMesh*)), SLOT(setActiveObject(QSegMesh*)), Qt::UniqueConnection);
	activeScene->connect(ui.actionExportObject, SIGNAL(triggered()), SLOT(exportActiveObject()), Qt::UniqueConnection);
	activeScene->connect(mDoc, SIGNAL(printMessage(QString)), SLOT(print(QString)), Qt::UniqueConnection);

	mDoc->connect(activeScene, SIGNAL(exportActiveObject(QSegMesh*)), SLOT(exportObject(QSegMesh*)), Qt::UniqueConnection);
}
