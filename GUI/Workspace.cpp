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

	// === Left dock
	// Stacker widget
	sp = new StackerPanel();
	leftLayout->addWidget(sp);

	// === Richt dock
	// Controller widget
	cp = new ControllerPanel();
	rightLayout->addWidget(cp);

	// Group widget
	gp = new GroupPanel();
	rightLayout->addWidget(gp);

	// Transformation widget
	tp = new TransformationPanel();
	rightLayout->addWidget(tp);

	// Deformer widget
	dp = new DeformerPanel();
	rightLayout->addWidget(dp);

	// Voxel Deformer widget
	vdp = new QVoxelDeformerPanel();
	rightLayout->addWidget(vdp);

	// Mesh info widget
	mi = new MeshInfoPanel();
	rightLayout->addWidget(mi);

	// Hide the right dock by default
	ui.rightDock->hide();


	// Create mesh document manager
	mDoc = new QMeshDoc(this);

	// Add new scene action
	connect(ui.actionNewScene, SIGNAL(triggered()), SLOT(addNewScene()));

	// Connect to mesh management
	connect(ui.actionImportObject, SIGNAL(triggered()), mDoc, SLOT(importObject()));

	// Connect to mesh browser
	connect(ui.actionMeshBrowser, SIGNAL(triggered()), mDoc, SLOT(importObjectBrowser()));

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

	// Focus changes in scene
	connect(newScene, SIGNAL(gotFocus(Scene*)), SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(gotFocus(Scene*)), tp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(gotFocus(Scene*)), mi, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(gotFocus(Scene*)), sp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(gotFocus(Scene*)), cp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(gotFocus(Scene*)), dp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(gotFocus(Scene*)), vdp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(gotFocus(Scene*)), gp, SLOT(setActiveScene(Scene*)));

	connect(newScene, SIGNAL(lostFocus(Scene*)), SLOT(disconnectScene(Scene*)));
	connect(newScene, SIGNAL(sceneClosed(Scene*)), SLOT(sceneClosed(Scene*)));
	connect(newScene, SIGNAL(sceneClosed(Scene*)), sp, SLOT(setActiveScene(Scene*)));

	// Objects changed in scene
	connect(newScene, SIGNAL(objectDiscarded(QString)), mDoc, SLOT(deleteObject(QString)));
	connect(newScene, SIGNAL(objectInserted()), sp, SLOT(updateActiveObject()));
	connect(newScene, SIGNAL(objectInserted()), cp, SLOT(updateController()));

	// Stack panel
	connect(sp, SIGNAL(printMessage(QString)), newScene, SLOT(print(QString)));
	connect(sp, SIGNAL(objectModified()), newScene, SLOT(updateActiveObject()));
	connect(sp, SIGNAL(objectModified()), cp, SLOT(updateController()));

	// Groups
	connect(newScene, SIGNAL(groupsChanged()), gp, SLOT(updateWidget()));

	// Object transformed by transformation panel
	connect(tp, SIGNAL(objectModified()), newScene, SLOT(updateActiveObject()));
	connect(tp, SIGNAL(objectModified()), sp, SLOT(updateActiveObject()));
	connect(tp, SIGNAL(objectModified()), cp, SLOT(updateController()));

	// Deformation
	connect(dp, SIGNAL(deformerCreated(QFFD *)), newScene, SLOT(setActiveDeformer(QFFD *)));
	connect(vdp, SIGNAL(deformerCreated(VoxelDeformer *)), newScene, SLOT(setActiveVoxelDeformer(VoxelDeformer *)));

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
