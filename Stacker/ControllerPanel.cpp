#include "ControllerPanel.h"

#include <fstream>

#include <QFileDialog>

#include "GUI/global.h"
#include "StackerGlobal.h"

#include "Primitive.h"
#include "Controller.h"
#include "ConstraintGraphVis.h"

ConstraintGraphVis * vis;

ControllerPanel::ControllerPanel( QWidget * parent /*= NULL*/ )
{
	controllerWidget.setupUi(this);
	
	// Connect button actions
	connect(controllerWidget.saveButton, SIGNAL(clicked()), SLOT(save()));
	connect(controllerWidget.loadButton, SIGNAL(clicked()), SLOT(load()));
	connect(controllerWidget.removeButton, SIGNAL(clicked()), SLOT(removeSelected()));
	connect(controllerWidget.clearButton, SIGNAL(clicked()), SLOT(clear()));
	connect(controllerWidget.showPrimitives, SIGNAL(stateChanged (int)), SLOT(togglePrimDisplay(int)));

	connect(controllerWidget.showGraph, SIGNAL(clicked()), SLOT(showGraph()));

	// Primitives
	connect(controllerWidget.skeletonJoints, SIGNAL(valueChanged(int)), this, SLOT(setSkeletonJoints(int)) );
	connect(controllerWidget.convertToGC, SIGNAL(clicked()), SLOT(convertGC()));
	connect(controllerWidget.convertToCuboid, SIGNAL(clicked()), SLOT(convertCuboid()));

	this->activeScene = NULL;

	this->layout()->addWidget(vis = new ConstraintGraphVis(new ConstraintGraph()));

	// Default values
	controllerWidget.skeletonJoints->setValue(GC_SKELETON_JOINTS_NUM);
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

	activeScene->activeObject()->ptr["controller"] = new Controller(activeScene->activeObject());

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
	Controller * ctrl = (Controller *)activeScene->activeObject()->ptr["controller"];
	ctrl->clearPrimitives();
}

Controller * ControllerPanel::controller()
{
	if(!activeScene || !activeScene->activeObject() || !(Controller *)activeScene->activeObject()->ptr["controller"])
		return NULL;

	return (Controller *)activeScene->activeObject()->ptr["controller"];
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

QSegMesh* ControllerPanel::activeObject()
{
	if (activeScene)
		return activeScene->activeObject();
	else 
		return NULL;
}


void ControllerPanel::setSkeletonJoints( int num )
{
	GC_SKELETON_JOINTS_NUM = num;
}

void ControllerPanel::convertGC()
{
	if(!activeObject() || !activeObject()->ptr["controller"]) return;

	Controller* ctrl = (Controller *)activeObject()->ptr["controller"];

	foreach(Primitive * prim, ctrl->getPrimitives())
	{
		if(prim->isSelected)
			ctrl->convertToGC(prim->id, !controllerWidget.basicFitGC->isChecked(), controllerWidget.convertGcAxis->value());
	}

	if(activeScene)	activeScene->updateGL();
}

void ControllerPanel::convertCuboid()
{
	if(!activeObject() || !activeObject()->ptr["controller"]) return;

	Controller* ctrl = (Controller *)activeObject()->ptr["controller"];

	foreach(Primitive * prim, ctrl->getPrimitives())
		if(prim->isSelected) ctrl->convertToCuboid(prim->id, controllerWidget.useAABB->isChecked(), controllerWidget.cuboidMethod->currentIndex());

	if(activeScene)	activeScene->updateGL();
}

void ControllerPanel::updateController()
{
	// Create a controller if non-exists
	if (activeScene && activeObject() && !activeObject()->ptr["controller"])
	{
		activeObject()->ptr["controller"] = new Controller(activeObject(), controllerWidget.useAABB->isChecked());
		activeScene->setSelectMode(CONTROLLER);
	}
}

void ControllerPanel::showGraph()
{
	if(controller())
	{
		vis->setGraph(new ConstraintGraph(controller()));
		vis->setFloating(true);
		vis->show();
	}
}

