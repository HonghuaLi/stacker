#include "Workspace.h"

#include <QVBoxLayout >

Workspace::Workspace(QWidget *parent, Qt::WFlags flags)	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	// New scene action
	connect(ui.actionNewScene, SIGNAL(triggered()), SLOT(addNewScene()));
	connect(ui.actionImportObject, SIGNAL(triggered()), SLOT(importObject()));

	stacker_panel = new StackerPanel();
	ui.leftDockWidget->setLayout(new QVBoxLayout);
	ui.leftDockWidget->layout()->addWidget(stacker_panel);

	// Create new scene when we start by default
	addNewScene();
}

Workspace::~Workspace()
{
	delete stacker_panel;
}

void Workspace::addNewScene()
{
	Scene * newScene = new Scene;

	ui.sceneArea->addSubWindow(newScene);

	newScene->showMaximized();
	newScene->setWindowTitle("Untitled");

	connect(newScene, SIGNAL(focusChanged(Scene*)), SLOT(sceneFocusChanged(Scene*)));

	connect(stacker_panel, SIGNAL(StackButtonClicked()), newScene, SLOT(doStacking()));
}

void Workspace::importObject()
{
	Scene * selectedScene = static_cast<Scene*>(ui.sceneArea->activeSubWindow()->widget());

	QString fileName = QFileDialog::getOpenFileName(this, tr("Insert Mesh"), "", tr("Mesh Files (*.obj *.off *.stl)"));

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
