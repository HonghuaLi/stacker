#include "ControllerPanel.h"

#include <fstream>

#include <QFileDialog>

#include "GUI/global.h"
#include "Primitive.h"
#include "Controller.h"
#include "ConstraintGraphVis.h"
#include "Numeric.h"

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

	// Gaussian 
	connect(controllerWidget.gaussianSlider, SIGNAL(valueChanged(int)), this, SLOT(setGaussianSigma(int)) );
	controllerWidget.gaussianSlider->setValue(GC_GAUSSIAN_SIGMA);

	this->activeScene = NULL;

	this->layout()->addWidget(vis = new ConstraintGraphVis(new ConstraintGraph()));

	// Default values
	if (ctrl())
		controllerWidget.skeletonJoints->setValue(ctrl()->GC_SKELETON_JOINTS_NUM);
	else
		controllerWidget.skeletonJoints->setValue(20);
}

void ControllerPanel::setActiveScene( Scene * newScene )
{
	this->activeScene = newScene;
}

void ControllerPanel::save()
{
	if(!ctrl())	return;

	QString fileName = QFileDialog::getSaveFileName(0, "Export Controller", DEFAULT_FILE_PATH, "Controller File (*.ctrl)"); 
	std::ofstream outF(qPrintable(fileName), std::ios::out);

	ctrl()->save(outF);

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

	ctrl()->load(inF);

	inF.close();

	DEFAULT_FILE_PATH = QFileInfo(fileName).absolutePath();

	activeScene->updateGL();

	emit(controllerModified());
}

void ControllerPanel::removeSelected()
{
	if(!ctrl())	return;

	Primitive * prim = ctrl()->getSelectedPrimitive();

	ctrl()->removePrimitive(prim);
	// delete prim?
}

void ControllerPanel::reset()
{
	if (activeScene->isEmpty()) return;

	if(!ctrl())
	{
		// Re-compute controller
		activeObject()->ptr["controller"] = new Controller(activeObject());
	}
	else
	{
		//delete controller();
		ctrl()->clearPrimitives();
		delete ctrl();
		activeObject()->ptr.erase( activeObject()->ptr.find("controller") );
	}

	// Refresh
	if (activeScene) activeScene->updateGL();
	emit(controllerModified());
}

Controller * ControllerPanel::ctrl()
{
	if(!activeScene || !activeObject())
		return NULL;

	return (Controller *)activeObject()->ptr["controller"];
}

void ControllerPanel::togglePrimDisplay(int state)
{
	if(!ctrl())	return;

	foreach(Primitive* prim, ctrl()->getPrimitives())
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
	if(!ctrl())	return;

	ctrl()->GC_SKELETON_JOINTS_NUM = num;
}

void ControllerPanel::convertGC()
{
	if(!ctrl())	return;

	foreach(Primitive * prim, ctrl()->getPrimitives())
	{
		if(prim->isSelected)
			ctrl()->convertToGC(prim->id, !controllerWidget.basicFitGC->isChecked(), controllerWidget.convertGcAxis->value());
	}

	activeScene->updateGL();
	emit(controllerModified());
}

void ControllerPanel::convertCuboid()
{
	if(!ctrl())	return;

	foreach(Primitive * prim, ctrl()->getPrimitives())
		if(prim->isSelected) ctrl()->convertToCuboid(prim->id, controllerWidget.useAABB->isChecked(), controllerWidget.cuboidMethod->currentIndex());

	activeScene->updateGL();
	emit(controllerModified());
}

void ControllerPanel::showGraph()
{
	if(!ctrl())	return;

	vis->setGraph(new ConstraintGraph(ctrl()));
	vis->setFloating(true);
	vis->show();
}

void ControllerPanel::setGaussianSigma( int step )
{
	GC_GAUSSIAN_SIGMA = (double)step / 10;; 
}

