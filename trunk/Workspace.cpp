#include "Workspace.h"

#include <QVBoxLayout>

Workspace::Workspace(QWidget *parent, Qt::WFlags flags)	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	ui.leftDockWidget->setLayout(new QVBoxLayout);
	ui.rightDockWidget->setLayout(new QVBoxLayout);

	// New scene action
	connect(ui.actionNewScene, SIGNAL(triggered()), SLOT(addNewScene()));
	connect(ui.actionImportObject, SIGNAL(triggered()), SLOT(importObject()));

	sp = new StackerPanel();
	ui.leftDockWidget->layout()->addWidget(sp);

	wp = new WiresPanel();
	ui.leftDockWidget->layout()->addWidget(wp);

	dp = new DeformerPanel();
	ui.rightDockWidget->layout()->addWidget(dp);

	// Create new scene when we start by default
	sceneCount = 0;
	addNewScene();
}

Workspace::~Workspace()
{
	
}

void Workspace::addNewScene()
{
	Scene * newScene;
	
	//main_args;
	QStringList mesh_fileNames = main_args.filter(QRegExp("^*(obj|off)$"));

	if(sceneCount == 0 && mesh_fileNames.size())
		newScene = new Scene(mesh_fileNames.first());
	else
		newScene = new Scene;

	ui.sceneArea->addSubWindow(newScene);

	newScene->showMaximized();
	newScene->setWindowTitle("Untitled");

	connect(newScene, SIGNAL(newSceneCreated()), sp, SLOT(sceneUpdated()));
	connect(newScene, SIGNAL(focusChanged(Scene*)), SLOT(sceneFocusChanged(Scene*)));

	connect(newScene, SIGNAL(focusChanged(Scene*)), wp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(focusChanged(Scene*)), sp, SLOT(setActiveScene(Scene*)));
	connect(newScene, SIGNAL(focusChanged(Scene*)), dp, SLOT(setActiveScene(Scene*)));

	// Objects inserted
	connect(newScene, SIGNAL(objectInserted(QSurfaceMesh *)), sp, SLOT(sceneUpdated()));

	// Wires
	connect(wp, SIGNAL(wiresFound(QVector<Wire>)), newScene, SLOT(setActiveWires(QVector<Wire>)));
	connect(wp, SIGNAL(wiresFound(QVector<Wire>)), newScene, SLOT(updateGL()));

	// Deformation
	connect(dp, SIGNAL(deformerCreated(QFFD *)), newScene, SLOT(setActiveDeformer(QFFD *))), 

	sceneCount++;
}

void Workspace::importObject()
{
	QMdiSubWindow * mdi_window = ui.sceneArea->activeSubWindow();
	if(!mdi_window) return;

	Scene * selectedScene = static_cast<Scene*>(mdi_window->widget());

	QString fileName = QFileDialog::getOpenFileName(this, "Insert Mesh", "", "Mesh Files (*.obj *.off *.stl)");

	if(fileName.length())
		selectedScene->insertObject(fileName);

	emit(importedObject(fileName));
}

void Workspace::sceneFocusChanged(Scene* scene)
{
	QString title = QString("%1 - %2")
		.arg(QFileInfo(QApplication::applicationFilePath()).baseName())
		.arg(scene->windowTitle());
}
