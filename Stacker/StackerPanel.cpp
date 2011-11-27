#include "StackerPanel.h"
#include "Contoller.h"
#include "Cuboid.h"
#include "ConvexHull3.h"
#include <QDockWidget>
#include <QVBoxLayout>
#include "Vector.h"
#include <fstream>
#include <algorithm>
#include <numeric>
#include <memory>

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	// Add controller deformer widget
	ctrlDeformer.setupUi(panel.controllerDeformerPanel);

	// Add a stacking preview widget
	stacker_preview = new StackerPreview(this);
	QDockWidget * previewDock = new QDockWidget();
	previewDock->setWidget (stacker_preview);
	panel.previewBox->layout()->addWidget(previewDock);

	// Offset function calculator
	hidden_viewer = new HiddenViewer();
	panel.groupBox->layout()->addWidget(hidden_viewer);
	activeOffset = new Offset(hidden_viewer);

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

	// Connect controller deformer
	connect(ctrlDeformer.transX, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.transY, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.transZ, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.rotX, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.rotY, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.rotZ, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.scaleX, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.scaleY, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.scaleZ, SIGNAL(valueChanged(int)), SLOT(updateController()));

	connect(ctrlDeformer.resetButton, SIGNAL(clicked()), SLOT(resetCtrlDeformerPanel()));
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

	// Get hot segments
	std::vector< uint > hotSegs = activeOffset->getHotSegments();
	int nHotSeg = hotSegs.size();

	// Initialization
	PrimitiveParamMap optimalParams;
	foreach (uint i, hotSegs) 
		optimalParams[i] = std::shared_ptr<PrimitiveParam> ( new CuboidParam );

	// Optimize
	double currE = sumEnergy();
	double step = 0.3;
	printf("Init energy = %f\n", currE);

	printf("\nStarting search");

	while(1)
	{
		double minE = DBL_MAX;
		
		// make deep copies..
		PrimitiveParamMap bestNeighborParams, currParams;
		currParams = optimalParams;

		// Check all the neighbors
		for (int i=0; i<nHotSeg; i++)
		{
			for (int j=0; j < currParams[i]->numParams(); j++)
			{
				double E = 0;

				// Forward
				currParams[i]->stepForward(j, step);
				ctrl->deformShape(currParams);
				emit(objectModified());
				E = sumEnergy();
				if (E < minE)
				{
					minE = E;
					bestNeighborParams = currParams;
				}
				currParams[i]->stepForward(j, -step);
				ctrl->recoverShape();
				printf("\n Stackability: %.3f Energy: %.3f\n", activeOffset->getStackability(), E);


				// Backward
				currParams[i]->stepForward(j, -step);
				ctrl->deformShape(currParams);
				emit(objectModified());
				E = sumEnergy();
				if (E < minE)
				{
					minE = E;
					bestNeighborParams = currParams;
				}
				currParams[i]->stepForward(j, step);
				ctrl->recoverShape();
				printf("\n Stackability: %.3f Energy: %.3f\n", activeOffset->getStackability(), E);
			}
		}

		printf(".");

		if(currE <= minE)
			// All neighbors are worse than the current state, stop
			break;
		else
		{
			// Update the current state the to best neighbor
			currE = minE;
			optimalParams = bestNeighborParams;		

			// Print current state
			printf("==============================\nThe current deformation parameters:\n\n");
			optimalParams.print();
			printf("Stackability: %.3f\n Energy: %.3f\n", activeOffset->getStackability(), currE);
		}

	}

	// Apply the optimal solution
	ctrl->deformShape(optimalParams);
	emit(objectModified());
	printf("\nOptimization is done ;)\n");

	// Print optimal solution
	optimalParams.print();
	printf("Stackability: %.3f\n Energy: %.3f\n", activeOffset->getStackability(), currE);
}

double StackerPanel::sumEnergy( )
{
	// Energy terms
	std::vector< double > E; 	
	
	// Compute the energy based on the original and current controllers
	Controller* ctrl = activeObject()->controller;
	int numPrimitive = ctrl->numPrimitives();

	Controller::Stat s1 = ctrl->getStat();
	Controller::Stat s0 = ctrl->getOriginalStat();

	// SHAPE BB:
	E.push_back(abs(s1.volumeBB - s0.volumeBB) / s0.volumeBB);


	// PART BB:
	double diffV = 0, sumV = 0;
	for(int i = 0; i < numPrimitive; i++)	{
		diffV += abs(s1.volumePrim[i] - s0.volumePrim[i]);
		sumV += s0.volumePrim[i];
	}
	E.push_back(diffV/sumV);


	// PROXMIITY:
	double diffP = 0, sumP = 0;
	for(uint i = 0; i <numPrimitive; i++)	{
		for (uint j = i + 1; j < numPrimitive; j++)		{
			diffP += abs(s1.proximity[std::make_pair(i,j)] - s0.proximity[std::make_pair(i,j)]);
			sumP += s0.proximity[std::make_pair(i,j)];
		}
	}
	E.push_back( sumP==0 ? diffP : diffP/sumP);

	// ROTATION
	double sumR = 0;
	for(int i = 0; i < numPrimitive; i++)	{	
		CuboidParam* params = (CuboidParam*) (&*(s1.params[i]));
		Vec3d R = params->getR();
		double diffR = ( abs(R[0]) + abs(R[1]) + abs(R[2]) ) / ( 3*360 );
		double w = s1.volumePrim[i];///s1.volumeBB;

		sumR += diffR ;
	}
	E.push_back( sumR );

	// SIGGRAPH 2012: last one... :D
	return  std::accumulate(E.begin(), E.end(), 0)- activeOffset->getStackability();
}

void StackerPanel::updateController()
{
	if(!activeObject() || !activeObject()->controller) return;

	Controller* ctrl = activeObject()->controller;
	ctrl->recoverShape();

	double scaling = 0.01;
	std::vector<double> vals(9);
	vals[0] = scaling * ctrlDeformer.transX->value();
	vals[1] = scaling * ctrlDeformer.transY->value();
	vals[2] = scaling * ctrlDeformer.transZ->value();

	vals[3] = scaling * ctrlDeformer.rotX->value();
	vals[4] = scaling * ctrlDeformer.rotY->value();
	vals[5] = scaling * ctrlDeformer.rotZ->value();

	vals[6] = scaling * ctrlDeformer.scaleX->value();
	vals[7] = scaling * ctrlDeformer.scaleY->value();
	vals[8] = scaling * ctrlDeformer.scaleZ->value();

	std::shared_ptr<PrimitiveParam> param( new CuboidParam );
	param->setParams(vals);

	for(uint i = 0; i < ctrl->numPrimitives(); i++){
		Cuboid * prim = (Cuboid *) ctrl->getPrimitive(i);

		if(prim->isSelected)
			prim->deform(param);
	}
	
	emit(objectModified());

	printf("Stackability: %.3f | Energy: %.3f\n", activeOffset->getStackability(), -99);
}

void StackerPanel::resetCtrlDeformerPanel()
{
	CuboidParam param;
	std::vector< double > defaultParams = param.getDefaulParam();

	ctrlDeformer.transX->setValue( defaultParams[0] * 100 );
	ctrlDeformer.transY->setValue( defaultParams[1] * 100 );	
	ctrlDeformer.transZ->setValue( defaultParams[2] * 100 );
	ctrlDeformer.rotX->setValue( defaultParams[3] * 100 );
	ctrlDeformer.rotY->setValue( defaultParams[4] * 100 );	
	ctrlDeformer.rotZ->setValue( defaultParams[5] * 100 );
	ctrlDeformer.scaleX->setValue( defaultParams[6] * 100 );
	ctrlDeformer.scaleY->setValue( defaultParams[7] * 100 );	
	ctrlDeformer.scaleZ->setValue( defaultParams[8] * 100 );

	emit(objectModified());
}
