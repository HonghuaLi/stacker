#include "global.h"
#include "Workspace.h"
#include <QVBoxLayout>
#include <QFileInfo>

Workspace::Workspace(QWidget *parent, Qt::WFlags flags)	: QMainWindow(parent, flags)
{
	activeScene = NULL;
	sceneCount = 0;

	ui.setupUi(this);

	QVBoxLayout * leftLayout = (QVBoxLayout *) ui.leftDockWidget->layout();
	QVBoxLayout * rightLayout = (QVBoxLayout *) ui.rightDockWidget->layout();

	// Default widgets
	tp = new TransformationPanel();
	rightLayout->addWidget(tp);
	
	mi = new MeshInfoPanel();
	rightLayout->addWidget(mi);

	// Stacker widgets
	sp = new StackerPanel();
	leftLayout->addWidget(sp);

	gp = new GroupPanel();
	rightLayout->addWidget(gp);

	cp = new ControllerPanel();
	rightLayout->addWidget(cp);

	dp = new DeformerPanel();
	rightLayout->addWidget(dp);

	vdp = new QVoxelDeformerPanel();
	rightLayout->addWidget(vdp);

	ui.rightDock->hide();

	// Create mesh document manager
	mDoc = new QMeshDoc(this);

	// Connect to mesh management
	connect(ui.actionImportObject, SIGNAL(triggered()), mDoc, SLOT(importObject()));

	// Add new scene action
	connect(ui.actionNewScene, SIGNAL(triggered()), SLOT(addNewScene()));

	leftLayout->addStretch();
	rightLayout->addStretch();
}

Workspace::~Workspace()
{
	
}

void Workspace::addNewScene()
{
	Scene * newScene = new Scene(this);

	ui.sceneArea->addSubWindow(newScene)->show();
	sceneCount++;

	newScene->showMaximized();
	newScene->setWindowTitle("Untitled");

	newScene->sp = sp;

	// == CONNECTIONS ==

	// Workspace window
	connect(newScene, SIGNAL(gotFocus(Scene*)), SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(lostFocus(Scene*)), SLOT(disconnectScene(Scene*)));
	connect(newScene, SIGNAL(sceneClosed(Scene*)), SLOT(sceneClosed(Scene*)));

	// MeshDoc
	connect(newScene, SIGNAL(objectDiscarded(QString)), mDoc, SLOT(deleteObject(QString)));

	// Object transformation
	connect(newScene, SIGNAL(gotFocus(Scene*)), tp, SLOT(setActiveScene(Scene*)));
	connect(tp, SIGNAL(objectModified()), newScene, SLOT(updateActiveObject()));
	connect(tp, SIGNAL(objectModified()), sp, SLOT(updateActiveObject()));
	connect(tp, SIGNAL(objectModified()), sp, SLOT(updateController()));

	// Object info panel
	connect(newScene, SIGNAL(gotFocus(Scene*)), mi, SLOT(setActiveScene(Scene*)));

	// Stack panel
	connect(newScene, SIGNAL(gotFocus(Scene*)), sp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(objectInserted()), sp, SLOT(updateActiveObject()));
	connect(newScene, SIGNAL(sceneClosed(Scene*)), sp, SLOT(setActiveScene(Scene*)));
	connect(sp, SIGNAL(printMessage(QString)), newScene, SLOT(print(QString)));
	connect(sp, SIGNAL(objectModified()), newScene, SLOT(updateActiveObject()));

	// Controllers panel
	connect(newScene, SIGNAL(gotFocus(Scene*)), cp, SLOT(setActiveScene(Scene*)));

	// Deformation
	connect(newScene, SIGNAL(gotFocus(Scene*)), dp, SLOT(setActiveScene(Scene*)));
	connect(dp, SIGNAL(deformerCreated(QFFD *)), newScene, SLOT(setActiveDeformer(QFFD *)));

	connect(newScene, SIGNAL(gotFocus(Scene*)), vdp, SLOT(setActiveScene(Scene*)));
	connect(vdp, SIGNAL(deformerCreated(VoxelDeformer *)), newScene, SLOT(setActiveVoxelDeformer(VoxelDeformer *)));

	// Groups
	connect(newScene, SIGNAL(gotFocus(Scene*)), gp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(groupsChanged()), gp, SLOT(updateWidget()));

	// == END ==

	setActiveScene(newScene);
}

void Workspace::setActiveScene(Scene* scene)
{
	QString title = QString("%1").arg(QFileInfo(QApplication::applicationFilePath()).baseName());
	
	activeScene = scene;

	if(activeScene)
	{
		// View operations
		connect(ui.actionCameraProjection, SIGNAL(triggered()), activeScene, SLOT(toggleCameraProjection()), Qt::UniqueConnection);
	
		// Connect mDoc
		activeScene->connect(ui.actionExportObject, SIGNAL(triggered()), SLOT(exportActiveObject()), Qt::UniqueConnection);
		activeScene->connect(mDoc, SIGNAL(objectImported(QSegMesh*)), SLOT(setActiveObject(QSegMesh*)), Qt::UniqueConnection);
		activeScene->connect(mDoc, SIGNAL(printMessage(QString)), SLOT(print(QString)), Qt::UniqueConnection);
		mDoc->connect(activeScene, SIGNAL(exportActiveObject(QSegMesh*)), SLOT(exportObject(QSegMesh*)), Qt::UniqueConnection);

		title += QString(" - %2").arg(scene->windowTitle());
	}

	// Set active scene
	tp->setActiveScene(activeScene);
	mi->setActiveScene(activeScene);
	sp->setActiveScene(activeScene);
	gp->setActiveScene(activeScene);
	cp->setActiveScene(activeScene);

	this->setWindowTitle(title);
}

void Workspace::disconnectScene(Scene* scene)
{
	mDoc->disconnect();
	ui.actionCameraProjection->disconnect();
}

void Workspace::sceneClosed( Scene* scene )
{
	int count = ui.sceneArea->subWindowList().size() - 1;

	if(count == 0)
	{
		setActiveScene(activeScene = NULL);
		printf("No scenes! %d\n", count);
	}
}
