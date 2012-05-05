#include "ControllerPanel.h"

#include <fstream>

#include <QFileDialog>

#include "GUI/global.h"
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
	connect(controllerWidget.resetButton, SIGNAL(clicked()), SLOT(reset()));
	connect(controllerWidget.showPrimitives, SIGNAL(stateChanged (int)), SLOT(togglePrimDisplay(int)));

	connect(controllerWidget.showGraph, SIGNAL(clicked()), SLOT(showGraph()));

	// Primitives
	connect(controllerWidget.skeletonJoints, SIGNAL(valueChanged(int)), this, SLOT(setSkeletonJoints(int)) );
	connect(controllerWidget.convertToGC, SIGNAL(clicked()), SLOT(convertGC()));
	connect(controllerWidget.convertToCuboid, SIGNAL(clicked()), SLOT(convertCuboid()));

	this->activeScene = NULL;

	this->layout()->addWidget(vis = new ConstraintGraphVis(new ConstraintGraph()));

	// Default values
	if (controller())
		controllerWidget.skeletonJoints->setValue(controller()->GC_SKELETON_JOINTS_NUM);
	else
		controllerWidget.skeletonJoints->setValue(20);
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


	QString fileName = QFileDialog::getOpenFileName(0, "Load Controller", DEFAULT_FILE_PATH, "Controller File (*.ctrl)"); 

	if(fileName.isEmpty() || !QFileInfo(fileName).exists()) return;

	std::ifstream inF(qPrintable(fileName), std::ios::in);

	controller()->load(inF);

	inF.close();

	DEFAULT_FILE_PATH = QFileInfo(fileName).absolutePath();

	if (activeScene) activeScene->updateGL();
}

void ControllerPanel::removeSelected()
{
	if(!controller())	return;

	Primitive * prim = controller()->getSelectedPrimitive();

	controller()->removePrimitive(prim);
	// delete prim?
}

void ControllerPanel::reset()
{
	Controller * ctrl = controller();
	if(!ctrl)
	{
		// Re-compute controller
		activeObject()->ptr["controller"] = new Controller(activeObject());
	}
	else
	{
		//delete controller();
		ctrl->clearPrimitives();
		delete ctrl;
		activeObject()->ptr.erase( activeObject()->ptr.find("controller") );
	}

	// Refresh
	if (activeScene) activeScene->updateGL();
}

Controller * ControllerPanel::controller()
{
	if(!activeScene || !activeObject())
		return NULL;

	return (Controller *)activeObject()->ptr["controller"];
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
	Controller * ctrl = controller();
	if (ctrl) ctrl->GC_SKELETON_JOINTS_NUM = num;
}

void ControllerPanel::convertGC()
{
	Controller * ctrl = controller();
	if (ctrl) 
	{
		foreach(Primitive * prim, ctrl->getPrimitives())
		{
			if(prim->isSelected)
				ctrl->convertToGC(prim->id, !controllerWidget.basicFitGC->isChecked(), controllerWidget.convertGcAxis->value());
		}

		activeScene->updateGL();
	}
}

void ControllerPanel::convertCuboid()
{
	if(!activeObject() || !activeObject()->ptr["controller"]) return;

	Controller* ctrl = (Controller *)activeObject()->ptr["controller"];

	foreach(Primitive * prim, ctrl->getPrimitives())
		if(prim->isSelected) ctrl->convertToCuboid(prim->id, controllerWidget.useAABB->isChecked(), controllerWidget.cuboidMethod->currentIndex());

	if(activeScene)	activeScene->updateGL();
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

