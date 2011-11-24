#include "StackerPanel.h"
#include "Contoller.h"
#include "ConvexHull3.h"
#include <QVBoxLayout>
#include "Vector.h"
#include <fstream>
#include <algorithm>

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	// Offset function calculator
	hidden_viewer = new HiddenViewer();
	panel.groupBox->layout()->addWidget(hidden_viewer);
	activeOffset = new Offset(hidden_viewer);

	// Add a stacking preview widget
	stacker_preview = new StackerPreview(this);
	QVBoxLayout *previewLayout = new QVBoxLayout(panel.previewBox);
	previewLayout->addWidget(stacker_preview);	

	// Connections
	connect(panel.offsetButton, SIGNAL(clicked()), SLOT(onOffsetButtonClicked()));
	connect(panel.controllerButton, SIGNAL(clicked()), SLOT(onControllerButtonClicked()));
	connect(panel.improveButton, SIGNAL(clicked()), SLOT(onImproveButtonClicked()));
	connect(panel.hotspotsButton, SIGNAL(clicked()), SLOT(onHotspotsButtonClicked()));
	connect(panel.convertToGC, SIGNAL(clicked()), SLOT(convertGC()));

	connect(this, SIGNAL(objectModified()), SLOT(updateActiveObject()));

	// Convex Hull
	connect(panel.chPrecision, SIGNAL(valueChanged (int)), this, SLOT(setConvexHullPrecision(int)));
	CH_PRECISION = panel.chPrecision->value();
}

void StackerPanel::onOffsetButtonClicked()
{
	if (!activeScene || activeScene->isEmpty())
	{
		emit(printMessage("There is no valid object."));
		return;
	}

	// compute offset
	activeOffset->computeOffset();
	activeOffset->saveOffsetAsImage("offset_image.png");
}

void StackerPanel::onControllerButtonClicked()
{
	if (!activeScene || activeScene->isEmpty())
	{
		emit(printMessage("There is no valid object."));
		return;
	}

	activeObject()->controller = new Controller(activeObject());
	activeObject()->controller->fitOBBs();

	activeScene->setSelectMode(CONTROLLER);

	showMessage("Controller is build for " + activeObject()->objectName());
}


void StackerPanel::onImproveButtonClicked()
{
	if (!activeScene || activeScene->isEmpty())	{
		showMessage("There is no valid object.");
		return;
	}

	if (!activeObject()->controller)	{
		showMessage("There is no controller built.");
		return;
	}


}



void StackerPanel::onHotspotsButtonClicked()
{
	activeOffset->detectHotspots();
	activeOffset->showHotVertices();
	emit(objectModified());
	showMessage("Hot spots are detected.");
}


void StackerPanel::setActiveScene( Scene * scene )
{
	if(activeScene != scene)
	{
		activeScene = scene;
		stacker_preview->setActiveScene(scene);
		hidden_viewer->setActiveScene(scene);
	}
}

void StackerPanel::updateActiveObject()
{
	activeOffset->computeOffset();	
	stacker_preview->updateActiveObject();
}

QSegMesh* StackerPanel::activeObject()
{
	if (activeScene)
		return activeScene->activeObject();
	else 
		return NULL;
}

void StackerPanel::showMessage( QString message )
{
	emit(printMessage(message));
}

void StackerPanel::setConvexHullPrecision( int p )
{
	CH_PRECISION = p;
}

void StackerPanel::convertGC()
{
	Controller* ctrl = activeObject()->controller;

	for(int i = 0; i < ctrl->numPrimitives(); i++){
		Primitive * prim = ctrl->getPrimitive(i);

		if(prim->isSelected)
			ctrl->convertToGC(prim->id, !panel.basicFitGC->isChecked());
	}
}

void StackerPanel::gradientDescentOptimize()
{
	Controller* ctrl = activeObject()->controller;
	int nSeg = ctrl->numHotPrimitives();

	// Initialization
	std::vector< cuboidDeformParam > params(nSeg);

	// 
	double currE = sumEnergy();
	double step = 0.02;
	while(1)
	{
		double minE = DBL_MAX;
		std::vector< cuboidDeformParam > bestParams = params;
		// check all the neighbours
		for (int i=0;i<nSeg;i++)
		{
			for (int j=0;j<9;j++)
			{
				params[i].stepForward(j, step);
				ctrl->deformShape(params);
				double E = sumEnergy();
				if (E < minE)
				{
					minE = E;
					bestParams = params;
				}

				params[i].stepForward(j, -step);


				params[i].stepForward(j, -step);
				ctrl->deformShape(params);
				double E = sumEnergy();
				if (E < minE)
				{
					minE = E;
					bestParams = params;
				}

				params[i].stepForward(j, step);

			}
		}

		if(currE <= minE)
			break;
	}
}

double StackerPanel::sumEnergy()
{
	// Compute the energy based on the original and current controllers
	return 0;
}

