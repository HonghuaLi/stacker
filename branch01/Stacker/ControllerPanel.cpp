#include "global.h"

#include "ControllerPanel.h"
#include <fstream>
#include <QFileDialog>

ControllerPanel::ControllerPanel( QWidget * parent /*= NULL*/ )
{
	controllerWidget.setupUi(this);
	
	// Connect button actions
	connect(controllerWidget.saveButton, SIGNAL(clicked()), SLOT(save()));
	connect(controllerWidget.loadButton, SIGNAL(clicked()), SLOT(load()));
	connect(controllerWidget.removeButton, SIGNAL(clicked()), SLOT(removeSelected()));
	connect(controllerWidget.clearButton, SIGNAL(clicked()), SLOT(clear()));
	connect(controllerWidget.showPrimitives, SIGNAL(stateChanged (int)), SLOT(togglePrimDisplay(int)));

	this->activeScene = NULL;
}

void ControllerPanel::setActiveScene( Scene * newScene )
{
	this->activeScene = newScene;
}

void ControllerPanel::save()
{
	if(!controller())	return;

	QString fileName = QFileDialog::getSaveFileName(0, "Export Controller", DEFAULT_FILE_PATH, "Controller File (*.ctrl)"); 
	std::ofstream outF(qPrintable(fileName), std::ios::out);

	controller()->save(outF);

	outF.close();

	DEFAULT_FILE_PATH = QFileInfo(fileName).absolutePath();
}

void ControllerPanel::load()
{
	if(!activeScene || !activeScene->activeObject())	return;

	activeScene->activeObject()->controller = new Controller(activeScene->activeObject());

	this->clear();

	QString fileName = QFileDialog::getOpenFileName(0, "Load Controller", DEFAULT_FILE_PATH, "Controller File (*.ctrl)"); 
	std::ifstream inF(qPrintable(fileName), std::ios::in);

	controller()->load(inF);

	inF.close();

	DEFAULT_FILE_PATH = QFileInfo(fileName).absolutePath();
}

void ControllerPanel::removeSelected()
{
	if(!controller())	return;

	Primitive * prim = controller()->getSelectedPrimitive();

	controller()->removePrimitive(prim);
	// delete prim?
}

void ControllerPanel::clear()
{
	if(!controller())	return;

	//delete controller();
	activeScene->activeObject()->controller->clearPrimitives();
}

Controller * ControllerPanel::controller()
{
	if(!activeScene || !activeScene->activeObject() || !activeScene->activeObject()->controller)
		return NULL;

	return activeScene->activeObject()->controller;
}

void ControllerPanel::togglePrimDisplay(int state)
{
	if(!controller())	return;

	foreach(Primitive* prim, controller()->getPrimitives())
	{
		prim->isDraw = state;
	}

	activeScene->updateGL();
}
