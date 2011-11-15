#include "Workspace.h"
#include <QVBoxLayout>

Workspace::Workspace(QWidget *parent, Qt::WFlags flags)	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	ui.leftDockWidget->setLayout(new QVBoxLayout);
	ui.rightDockWidget->setLayout(new QVBoxLayout);

	sp = new StackerPanel();
	ui.leftDockWidget->layout()->addWidget(sp);

	wp = new WiresPanel();
	ui.leftDockWidget->layout()->addWidget(wp);

	dp = new DeformerPanel();
	ui.rightDockWidget->layout()->addWidget(dp);

	// Create MeshDoc, where stores all the meshes
	mDoc = new QMeshDoc();
	connect(this ,SIGNAL(importObject(QString)), mDoc, SLOT(importObject(QString)));

	// New scene action
	connect(ui.actionNewScene, SIGNAL(triggered()), SLOT(addNewScene()));
	connect(ui.actionImportObject, SIGNAL(triggered()), SLOT(importObject()));

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
	connect(mDoc, SIGNAL(objectImported(QSegMesh*)), newScene, SLOT(setActiveObject(QSegMesh*)));
	connect(mDoc, SIGNAL(printMessage(QString)), newScene, SLOT(print(QString)));
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

	// Update stacker panel
	sp->setActiveScene(newScene);
}

void Workspace::importObject()
{
	QMdiSubWindow * mdi_window = ui.sceneArea->activeSubWindow();
	if(!mdi_window) return;

	Scene * selectedScene = static_cast<Scene*>(mdi_window->widget());

	QString fileName = QFileDialog::getOpenFileName(this, "Insert Mesh", "", "Mesh Files (*.obj *.off *.stl)");

	if (!fileName.isEmpty())
		emit(importObject(fileName));
}

void Workspace::setActiveScene(Scene* scene)
{
	QString title = QString("%1 - %2")
		.arg(QFileInfo(QApplication::applicationFilePath()).baseName())
		.arg(scene->windowTitle());

	this->setWindowTitle(title);
}