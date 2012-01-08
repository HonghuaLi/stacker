#include "StackerPanel.h"
#include "Controller.h"
#include "Cuboid.h"
#include "ConvexHull3.h"
#include <QDockWidget>
#include <QVBoxLayout>
#include "Vector.h"
#include <fstream>
#include <algorithm>
#include <numeric>
#include <QFile>
#include <QDir>
#include "Macros.h"
#include "GCylinder.h"

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	// Add controller deformer widget
	//ctrlDeformer.setupUi(panel.controllerDeformerPanel);

	QGridLayout * layout = (QGridLayout *) panel.groupBox->layout();
	int row = layout->rowCount() + 1;

	// Add a stacking preview widget
	stacker_preview = new StackerPreview(this);
	QDockWidget * previewDock = new QDockWidget("Preview");
	previewDock->setWidget (stacker_preview);
	layout->addWidget(previewDock, row++, 0,1,3);
	stacker_preview->setMinimumHeight(150);

	// Offset function calculator
	hidden_viewer = new HiddenViewer();
	QDockWidget * hiddenDock = new QDockWidget("Hidden");
	hiddenDock->setWidget (hidden_viewer);
	layout->addWidget(hiddenDock, row++, 0,1,3);
	hiddenDock->setFloating(true);
	hiddenDock->setWindowOpacity(0.0);

	activeOffset = new Offset(hidden_viewer);

	// Connections
	connect(panel.offsetButton, SIGNAL(clicked()), SLOT(onOffsetButtonClicked()));
	connect(panel.improveButton, SIGNAL(clicked()), SLOT(onImproveButtonClicked()));
	connect(panel.hotspotsButton, SIGNAL(clicked()), SLOT(onHotspotsButtonClicked()));
	connect(panel.solutionButton, SIGNAL(clicked()), SLOT(onSolutionButtonClicked()));
	connect(panel.convertToGC, SIGNAL(clicked()), SLOT(convertGC()));
	connect(panel.convertToCuboid, SIGNAL(clicked()), SLOT(convertCuboid()));
	connect(panel.radioController, SIGNAL(clicked()), SLOT(selectModeController()));
	connect(panel.radioControllerElement, SIGNAL(clicked()), SLOT(selectModeControllerElement()));

	connect(this, SIGNAL(objectModified()), SLOT(updateActiveObject()));

	// Convex Hull
	connect(panel.chPrecision, SIGNAL(valueChanged (int)), this, SLOT(setConvexHullPrecision(int)));
	CH_PRECISION = panel.chPrecision->value();

	// Offset 
	connect(panel.hotRange, SIGNAL(valueChanged (double)), this, SLOT(setHotRange(double)));

	// Voxel size
	connect(panel.jointsThreshold, SIGNAL(valueChanged(double)), this, SLOT(setJointThreshold(double)) );

	// GC
	connect(panel.skeletonJoints, SIGNAL(valueChanged(int)), this, SLOT(setSkeletonJoints(int)) );

	// Stacking
	connect(panel.stackCount, SIGNAL(valueChanged(int)), this, SLOT(setStackCount(int)) );

	// Connect controller deformer
	/*connect(ctrlDeformer.transX, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.transY, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.transZ, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.rotX, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.rotY, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.rotZ, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.scaleX, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.scaleY, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.scaleZ, SIGNAL(valueChanged(int)), SLOT(updateController()));
	connect(ctrlDeformer.resetButton, SIGNAL(clicked()), SLOT(resetCtrlDeformerPanel()));*/
	
	connect(panel.hidderViewerSize, SIGNAL(valueChanged(int)), hidden_viewer, SLOT(setResolution(int)));

	// Joints between primitives
	connect(panel.findJointsButton, SIGNAL(clicked()), SLOT(findJoints()));
	connect(panel.findPairwiseJointsButton, SIGNAL(clicked()), SLOT(findPairwiseJoints()));

	// Paper stuff
	connect(panel.outputButton, SIGNAL(clicked()), SLOT(outputForPaper()));

	// Stacking direction
	connect(panel.searchDirectionButton, SIGNAL(clicked()), SLOT(searchDirection()));
}

void StackerPanel::onOffsetButtonClicked()
{
	if (!activeScene || activeScene->isEmpty())
	{
		emit(printMessage("There is no valid object."));
		return;
	}

	// compute offset
	activeOffset->computeOffsetOfShape();
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

	activeOffset->improveStackabilityTo(panel.targetS->value());
	emit(objectModified());
}

void StackerPanel::onHotspotsButtonClicked()
{
	if(!activeScene || !activeObject())
		return;

	activeOffset->detectHotspots();
	activeOffset->showHotSpots();
	//emit(objectModified());
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
	if (panel.rotAroundAxis->isChecked())
		activeOffset->computeOffsetOfShape(panel.rotStackingDensity->value(), true);
	else if (panel.rotFreeForm->isChecked())

		activeOffset->computeOffsetOfShape();

	stacker_preview->stackCount = panel.stackCount->value();
	stacker_preview->updateActiveObject();

	if (activeObject()->controller == NULL)
	{
		activeObject()->controller = new Controller(activeObject(), panel.useAABB->isChecked());
		activeScene->setSelectMode(CONTROLLER);
		showMessage("Controller is built for " + activeObject()->objectName());
	}

	std::cout << "objectH = " << activeOffset->objectH << std::endl;
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

	if (!activeScene || activeScene->isEmpty()){
		emit(printMessage("There is no valid object."));
		return;
	}

	activeObject()->controller = new Controller(activeObject(), panel.useAABB->isChecked());

	activeScene->setSelectMode(CONTROLLER);

	showMessage("Controller is built for " + activeObject()->objectName());
}

void StackerPanel::convertGC()
{
	if(!activeObject() || !activeObject()->controller) return;

	Controller* ctrl = activeObject()->controller;

	foreach(Primitive * prim, ctrl->getPrimitives())
	{
		if(prim->isSelected)
			ctrl->convertToGC(prim->id, !panel.basicFitGC->isChecked(), panel.convertGcAxis->value());
	}
}

void StackerPanel::convertCuboid()
{
	if(!activeObject() || !activeObject()->controller) return;

	Controller* ctrl = activeObject()->controller;

	foreach(Primitive * prim, ctrl->getPrimitives())
		if(prim->isSelected) ctrl->convertToCuboid(prim->id, panel.useAABB->isChecked(), panel.cuboidMethod->currentIndex());
}

void StackerPanel::gradientDescentOptimize()
{
	Controller* ctrl = activeObject()->controller;

	// Get hot segments
	activeOffset->detectHotspots();
	std::set< QString > hotSegs = activeOffset->getHotSegment();
	int nHotSeg = hotSegs.size();

	// Initialization
	PrimitiveParamMap optimalParams;
	foreach (QString sid, hotSegs) 
	{
		optimalParams[sid] = (PrimitiveParam*) new CuboidParam(sid);
	}

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

		// Check all the neighbors in the deformation space
		for (std::set< QString >::iterator i=hotSegs.begin(); i!=hotSegs.end(); i++)
		{
			QString sid = *i;

			for (int j=0; j < currParams[sid]->numParams(); j++)
			{
				double E = 0;

				// Forward
				currParams[sid]->stepForward(j, step);
				ctrl->deformShape(currParams);
				emit(objectModified());
				E = sumEnergy();
				if (E < minE)
				{
					minE = E;
					bestNeighborParams = currParams;
				}
				currParams[sid]->stepForward(j, -step);
				ctrl->recoverShape();
				printf("\n Stackability: %.3f Energy: %.3f\n", activeOffset->getStackability(), E);


				// Backward
				currParams[sid]->stepForward(j, -step);
				ctrl->deformShape(currParams);
				emit(objectModified());
				E = sumEnergy();
				if (E < minE)
				{
					minE = E;
					bestNeighborParams = currParams;
				}
				currParams[sid]->stepForward(j, step);
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
			printf("Stackability: %.3f Energy: %.3f\n", activeOffset->getStackability(), currE);
		}

	}

	// Apply the optimal solution
	ctrl->deformShape(optimalParams);
	emit(objectModified());
	printf("\n===\nOptimization is done ;)\n");

	// Print optimal solution
	//optimalParams.print();
	printf("\nStackability: %.3f Energy: %.3f\n", activeOffset->getStackability(), currE);
}

double StackerPanel::sumEnergy( )
{
	// Energy terms
	std::vector< double > E; 	
	
	// Compute the energy based on the original and current controllers
	Controller* ctrl = activeObject()->controller;
	int numPrimitive = ctrl->numPrimitives();

	// NOTE: struct Stat cannot deep copy params, so have to use reference here!!!
	Controller::Stat& s1 = ctrl->getStat();
	Controller::Stat& s0 = ctrl->getOriginalStat();

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
		CuboidParam* params = (CuboidParam*) (s1.params[ctrl->primitiveIdNum[i]]);
		Vec3d R = params->getR();
		double diffR = ( abs(R[0]) + abs(R[1]) + abs(R[2]) ) / ( 3*360 );
		double w = s1.volumePrim[i];///s1.volumeBB;

		sumR += diffR ;
	}
	E.push_back( sumR );

	// SIGGRAPH 2012: last one... :D
	return Sum(E)- activeOffset->getStackability();
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

	PrimitiveParam* param = ( CuboidParam* ) new CuboidParam("");
	param->setParams(vals);

	for(uint i = 0; i < ctrl->numPrimitives(); i++){
		Cuboid * prim = (Cuboid *) ctrl->getPrimitive(ctrl->primitiveIdNum[i]);

		if(prim->isSelected)
			prim->deform(param);
	}
	
	emit(objectModified());

	printf("Stackability: %.3f | Energy: %.3f\n", activeOffset->getStackability(), 0.0);
}

void StackerPanel::resetCtrlDeformerPanel()
{
	CuboidParam param("");
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

void StackerPanel::selectModeController()
{
	if(!activeScene || !activeObject() || !activeObject()->controller)	return;
	
	activeScene->setSelectMode(CONTROLLER);
}


void StackerPanel::selectModeControllerElement()
{
	if(!activeScene || !activeObject() || !activeObject()->controller)	return;

	activeScene->setSelectMode(CONTROLLER_ELEMENT);
}


void StackerPanel::findJoints()
{
	if(!activeScene || !activeObject() || !activeObject()->controller)	return;
	
	activeObject()->controller->findJoints();
}

void StackerPanel::findPairwiseJoints()
{
	if(!activeScene || !activeObject() || !activeObject()->controller)	return;

	if (activeScene->selection.size() < 2) return;

	Controller* ctrl = activeObject()->controller;

	int selID1 = activeScene->selection[0];
	int selID2 = activeScene->selection[1];

	activeObject()->controller->findPairwiseJoints(ctrl->primitiveIdNum[selID1],ctrl->primitiveIdNum[selID2], panel.numJoints->value());
}
void StackerPanel::onSolutionButtonClicked()
{
	activeOffset->showSolution(panel.hsID->value());
	emit(objectModified());
}

void StackerPanel::setHotRange( double range)
{
	activeOffset->HOT_RANGE = range;
}

void StackerPanel::outputForPaper()
{
	// Make directory and set file paths
	QString outputName = panel.outputFileName->text();
	QString currDirectory = QDir::currentPath();
	QString exportDir = currDirectory + "/" + outputName;
	QDir().mkdir(exportDir);

	QMap<QString, QString> data;

	data["srcObj"] = activeObject()->objectName();
	data["upperEnvelope"] = "envelope_upper.png";
	data["lowerEnvelope"] = "envelope_lower.png";
	data["offsetImg"] = "offset_function.png";
	data["offsetData"] = "offset_function.dat";

	// Compute everything
	activeOffset->computeOffsetOfShape();
	

	// 1) Save envelopes + offset function (both image + values)
	double maxUE = Offset::getMaxValue(activeOffset->upperEnvelope);
	double minLE = Offset::getMinValue(activeOffset->lowerEnvelope);

	Offset::saveAsImage(activeOffset->upperEnvelope, maxUE, exportDir + "/" + data["upperEnvelope"]);
	Offset::saveAsImage(activeOffset->lowerEnvelope, minLE, exportDir + "/" + data["lowerEnvelope"]);

	Offset::saveAsImage(activeOffset->offset, activeOffset->O_max, exportDir + "/" + data["offsetImg"]);
	Offset::saveAsData(activeOffset->offset, 1.0, exportDir + "/" + data["offsetData"]);
	data["O_max"] = QString::number(activeOffset->O_max);
	data["Stackability"] = QString::number(activeOffset->getStackability());


	// 2) Save meshes stacked
	data["stackPreview"] = "stackPreview.obj";
	stacker_preview->saveStackObj(exportDir + "/" + data["stackPreview"], 3, 1.0);


	// 3) Save stuck points / region
	data["upperStuck"] = "upperStuck.dat";
	data["lowerStuck"] = "lowerStuck.dat";

	activeOffset->HOT_RANGE = 0.95;
	activeOffset->detectHotspots();
	while (activeOffset->upperHotSpots.empty()){
		activeOffset->HOT_RANGE -= 0.05;
		activeOffset->detectHotspots();
	}

	activeOffset->detectHotspots();
	double sampleSize = 1.0; // % 5
	activeOffset->saveHotSpots(exportDir + "/" + data["upperStuck"], 1, sampleSize);
	activeOffset->saveHotSpots(exportDir + "/" + data["lowerStuck"], -1, sampleSize);

	// 4) Save stacking direction
	Vec3d direction = stacker_preview->stackDirection;
	data["stackDir"] = QString("%1 %2 %3").arg(direction.x()).arg(direction.y()).arg(direction.z());


	// Write info file
	QFile infoFile(exportDir + "/" + QString("info.txt"));
	infoFile.open(QIODevice::WriteOnly | QIODevice::Text);

	QMapIterator<QString, QString> i(data);
	while (i.hasNext()) {
		i.next();
		infoFile.write(qPrintable(i.key().leftJustified(14) + "\t" + i.value() + "\n"));
	}
	infoFile.close();
}

void StackerPanel::setJointThreshold( double threshold )
{
	JOINT_THRESHOLD = threshold;
}

void StackerPanel::searchDirection()
{
	int num = panel.searchDensity->value();
	int angleNum = panel.searchAngleDensity->value();
	int num2 = num;		// maybe num2 = num * 2 for bias
	double r = 1.0;

	double x,y,z, theta = 0, phi = 0;

	double deltaTheta = M_PI / num;
	double deltaPhi = 2.0 * M_PI / num2;
	
	std::vector<Vec3d> samples;

	if(panel.searchAlongAxis->isChecked())
	{
		deltaPhi = M_PI;
	}

	for(uint i = 0; i <= num2 && phi < 2 * M_PI; i++){
		for(uint j = 0; j <= num; j++){
			x = r * cos(phi) * sin(theta);
			y = r * sin(phi) * sin (theta);
			z = r * cos(theta);

			// Sample
			Vec3d v(x,y,z);
			samples.push_back(v.normalized());

			theta += deltaTheta;
		}

		phi += deltaPhi;
		theta = 0;
	}

	if(panel.searchAlongAxis->isChecked()){
		for(int i = 0; i < samples.size(); i++){
			switch(panel.searchAxis->value())
			{
			case 0: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(1,0,0)); break;
			case 1: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(0,1,0)); break;
			case 2: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(0,0,1)); break;
			}

			activeObject()->getSegment(0)->debug_points.push_back(samples[i]);
		}
	}
	
	int sampleCount = 0;
	double maxStackbaility = activeOffset->computeOffsetOfShape(); // don't go worse than start
	int bestIndex = 0;
	double bestAngle = 0;

	printf("Start stackability = %f \n", maxStackbaility);

	QElapsedTimer timing; timing.start();

	for(int i = 0; i < samples.size(); i++){
		activeObject()->rotateUp(samples[i]);

		double omega = M_PI / angleNum;

		for(double theta = 0; theta <= 2 * M_PI; theta += omega)
		{
			activeObject()->rotateAroundUp(omega);

			double stackability = activeOffset->computeOffsetOfShape();
			
			if(stackability > maxStackbaility)
			{
				bestIndex = i;
				bestAngle = theta;
				maxStackbaility = stackability;
			}

			sampleCount++;
		}

		// restore back
		activeObject()->rotateUp(Vec3d(0,0,1));

		// report percent
		int progress = (double(i) / samples.size()) * 100;

		printf("progress: %d %%  \t best = %.3f \r", progress, maxStackbaility);
	}

	std::cout << "\r" << "Done.\n";

	double bestX = samples[bestIndex].x();
	double bestY = samples[bestIndex].y();
	double bestZ = samples[bestIndex].z();

	printf("\nSampled (%d) samples for best direction.\n", sampleCount);
	printf("Best stackability = %f\t direction %.3f,%.3f,%.3f\n", maxStackbaility, bestX,bestY,bestZ);
	printf("Time (%d ms).", timing.elapsed());

	activeObject()->rotateUp(samples[bestIndex]);

	emit(objectModified());
}

void StackerPanel::setSkeletonJoints( int num )
{
	skeletonJoints = num;
}

void StackerPanel::setStackCount( int num )
{
	stacker_preview->stackCount = num;
	stacker_preview->updateActiveObject();
}



