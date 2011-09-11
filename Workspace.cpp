#include "Workspace.h"

#include <QVBoxLayout>

Workspace::Workspace(QWidget *parent, Qt::WFlags flags)	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	ui.leftDockWidget->setLayout(new QVBoxLayout);

	// New scene action
	connect(ui.actionNewScene, SIGNAL(triggered()), SLOT(addNewScene()));
	connect(ui.actionImportObject, SIGNAL(triggered()), SLOT(importObject()));

	sp = new StackerPanel();
	ui.leftDockWidget->layout()->addWidget(sp);

	wp = new WiresPanel();
	ui.leftDockWidget->layout()->addWidget(wp);

	// Create new scene when we start by default
	addNewScene();
}

Workspace::~Workspace()
{
	
}

void Workspace::addNewScene()
{
	Scene * newScene = new Scene;

	ui.sceneArea->addSubWindow(newScene);

	newScene->showMaximized();
	newScene->setWindowTitle("Untitled");

	connect(newScene, SIGNAL(focusChanged(Scene*)), SLOT(sceneFocusChanged(Scene*)));
	connect(newScene, SIGNAL(focusChanged(Scene*)), wp, SLOT(setActiveScene(Scene*)));

	// Stacker
	connect(sp, SIGNAL(StackButtonClicked()), newScene, SLOT(doStacking()));

	// Wires
	connect(wp, SIGNAL(wiresFound(QVector<Wire>)), newScene, SLOT(setActiveWires(QVector<Wire>)));
}

void Workspace::importObject()
{
	Scene * selectedScene = static_cast<Scene*>(ui.sceneArea->activeSubWindow()->widget());

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
