#include "StackerPanel.h"
#include "Contoller.h"
#include "ConvexHull3.h"
#include <QDockWidget>
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

	QDockWidget * previewDock = new QDockWidget();
	previewDock->setWidget (stacker_preview);
	panel.previewBox->layout()->addWidget(previewDock);

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

	gradientDescentOptimize();
}

void StackerPanel::onHotspotsButtonClicked()
{
	if(!activeScene || !activeObject())
		return;

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
	if(!activeObject() || !activeObject()->controller) return;

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
	int nSeg = ctrl->numPrimitives();

	// Initialization
	std::vector< cuboidDeformParam > optimalParams(nSeg);
	//optimalParams[0].randomSample();
	//optimalParams[1].randomSample();

	originalStats = ctrl->getStat();

	// Optimize
	double currE = sumEnergy();
	double step = 0.3;
	printf("Init energy = %f\n", currE);

	printf("\nStarting search");

	while(1)
	{
		double minE = DBL_MAX;
		
		std::vector< cuboidDeformParam > bestNeighborParams, currParams;
		bestNeighborParams = currParams = optimalParams;


		// check all the neighbors
		for (int i=0;i<nSeg;i++)
		{
			for (int j=0;j<9;j++)
			{
				double E = 0;

				// Forward
				currParams[i].stepForward(j, step);
				ctrl->deformShape(currParams);
				emit(objectModified());
				E = sumEnergy();
				if (E < minE)
				{
					minE = E;
					bestNeighborParams = currParams;
				}
				currParams[i].stepForward(j, -step);
				ctrl->recoverShape();
				printf("\n Stackability: %.3f Energy: %.3f\n", activeOffset->getStackability(), E);


				// Backward search
				currParams[i].stepForward(j, -step);
				ctrl->deformShape(currParams);
				emit(objectModified());
				E = sumEnergy();
				if (E < minE)
				{
					minE = E;
					bestNeighborParams = currParams;
				}
				currParams[i].stepForward(j, step);
				ctrl->recoverShape();
				printf("\n Stackability: %.3f Energy: %.3f\n", activeOffset->getStackability(), E);
			}
		}

		printf(".");

		if(currE <= minE)
			break;
		else
		{
			currE = minE;
			optimalParams = bestNeighborParams;		
			ctrl->deformShape(optimalParams);
			emit(objectModified());

			// Print current best solution
			printf("==============================\nThe current parameters:\n\n");
			optimalParams[0].print();
			printf("\n");
			optimalParams[1].print();
			printf("\n Stackability: %.3f\n Energy: %.3f\n", activeOffset->getStackability(), currE);
		}

	}

	// Apply the optimal solution
	ctrl->deformShape(optimalParams);
	emit(objectModified());
	printf("\nOptimization is done ;)\n");

	// Print optimal solution
	printf("==============================\nThe current parameters:\n\n");
	optimalParams[0].print();
	printf("\n");
	optimalParams[1].print();
	printf("\n Stackability: %.3f\n Energy: %.3f\n", activeOffset->getStackability(), sumEnergy());
}

double StackerPanel::sumEnergy()
{
	// Compute the energy based on the original and current controllers

	Controller* ctrl = activeObject()->controller;
	
	std::vector<double> penalties = ctrl->difference( originalStats );

	// SIGGRAPH 2012: last siggraph... :D
	return penalties[0] + penalties[1] + 0 * penalties[2] - activeOffset->getStackability();
}

