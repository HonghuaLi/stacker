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
#include <QFileDialog>
#include <QDesktopWidget>
#include "global.h"

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
	hiddenDock->setWindowOpacity(1.0);

	int x = qApp->desktop()->availableGeometry().width();
	hiddenDock->move(QPoint(x - hiddenDock->width(),0));
	
	activeOffset = new Offset(hidden_viewer);

	// Buttons
	connect(panel.offsetButton, SIGNAL(clicked()), SLOT(onOffsetButtonClicked()));
	connect(panel.improveButton, SIGNAL(clicked()), SLOT(onImproveButtonClicked()));
	connect(panel.hotspotsButton, SIGNAL(clicked()), SLOT(onHotspotsButtonClicked()));
	connect(panel.suggestButton, SIGNAL(clicked()), SLOT(onSuggestButtonClicked()));
	connect(panel.saveSuggestionsButton, SIGNAL(clicked()), SLOT(onSaveSuggestionsButtonClicked()));
	connect(panel.loadSuggestionsButton, SIGNAL(clicked()), SLOT(onLoadSuggestionsButtonClicked()));
	connect(panel.convertToGC, SIGNAL(clicked()), SLOT(convertGC()));
	connect(panel.convertToCuboid, SIGNAL(clicked()), SLOT(convertCuboid()));

	connect(this, SIGNAL(objectModified()), SLOT(updateActiveObject()));

	// Parameters
	connect(panel.chPrecision, SIGNAL(valueChanged (int)), this, SLOT(setConvexHullPrecision(int)));
	CH_PRECISION = panel.chPrecision->value();
	connect(panel.hotRange, SIGNAL(valueChanged (double)), this, SLOT(setHotRange(double)));
	connect(panel.jointsThreshold, SIGNAL(valueChanged(double)), this, SLOT(setJointThreshold(double)) );
	connect(panel.skeletonJoints, SIGNAL(valueChanged(int)), this, SLOT(setSkeletonJoints(int)) );
	connect(panel.stackCount, SIGNAL(valueChanged(int)), this, SLOT(setStackCount(int)) );
	connect(panel.solutionID, SIGNAL(valueChanged(int)), this, SLOT(setSolutionID(int)));
	connect(panel.suggestionID, SIGNAL(valueChanged(int)), this, SLOT(setSuggestionID(int)));
	connect(panel.targetS, SIGNAL(valueChanged(double)), this, SLOT(setTargetStackability(double)));
	connect(panel.normalzieMesh, SIGNAL(clicked()), this, SLOT(onNomalizeMeshChecked()));
	connect(panel.moveCenterToOrigin, SIGNAL(clicked()), this, SLOT(onMoveCenterToOriginChecked()));

	connect(panel.BBTolerance, SIGNAL(valueChanged(double)), this, SLOT(setBBTolerance(double)) );
	connect(panel.numExpectedSolutions, SIGNAL(valueChanged(int)), this, SLOT(setNumExpectedSolutions(int)) );
	
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

	activeOffset->improveStackabilityToTarget();

	int total = activeOffset->solutions.size();
	panel.numSolution->setText(QString("/ %1").arg(total));
	panel.suggestionID->setValue(0);

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
		activeOffset->computeOffsetOfShape( ROT_AROUND_AXIS, panel.rotStackingDensity->value() );
	else if (panel.rotFreeForm->isChecked())
		activeOffset->computeOffsetOfShape( ROT_FREE_FORM, panel.rotStackingDensity->value() );
	else
		activeOffset->computeOffsetOfShape();

	stacker_preview->stackCount = panel.stackCount->value();
	stacker_preview->updateActiveObject();

	if (activeScene && activeObject() && activeObject()->controller == NULL)
	{
		activeObject()->controller = new Controller(activeObject(), panel.useAABB->isChecked());
		activeScene->setSelectMode(CONTROLLER);
		showMessage("Controller is built for " + activeObject()->objectName());
	}

	//std::cout << "objectH = " << activeOffset->objectH << std::endl;
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


void StackerPanel::setSolutionID(int id)
{
	int total = activeOffset->solutions.size();
	panel.numSolution->setText(QString("/ %1").arg(total));

	id = (0==total)? 0 : (id % total);
	panel.solutionID->setValue(id);
	activeOffset->showSolution(id);

	emit(objectModified());
}

void StackerPanel::setSuggestionID(int id)
{
	int total = activeOffset->suggestSolutions.size();
	panel.numSuggestion->setText(QString("/ %1").arg(total));

	id = (0==total)? 0 : (id % total);
	panel.suggestionID->setValue(id);
	activeOffset->showSuggestion(id);

	emit(objectModified());
}

void StackerPanel::setHotRange( double range)
{
	HOT_RANGE = range;
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

	HOT_RANGE = 0.95;
	activeOffset->detectHotspots();
	while (activeOffset->upperHotSpots.empty()){
		HOT_RANGE -= 0.05;
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

void StackerPanel::onSuggestButtonClicked()
{
	activeScene->suggestions.clear();
	activeScene->suggestions = activeOffset->getSuggestions();
	
	int total = activeOffset->suggestSolutions.size();
	panel.numSuggestion->setText(QString("/ %1").arg(total));
	panel.suggestionID->setValue(0);
	emit(objectModified());
}

void StackerPanel::onSaveSuggestionsButtonClicked()
{
	if(!activeScene || !activeObject())	return;

	QVector< EditSuggestion > &suggestions = activeScene->suggestions;
	if (suggestions.empty()) return;

	QString fileName = QFileDialog::getSaveFileName(0, "Export Groups", "", "Group File (*.sgt)"); 
	std::ofstream outF(qPrintable(fileName), std::ios::out);

	outF << suggestions.size() << std::endl;
	foreach(EditSuggestion sgt, suggestions)
	{
		outF << sgt.center << '\t' << sgt.direction << '\t' << sgt.value << std::endl;
	}
	outF.close();
}

void StackerPanel::onLoadSuggestionsButtonClicked()
{
	if(!activeScene)	return;

	QString fileName = QFileDialog::getOpenFileName(0, "Import Groups", "", "Group File (*.sgt)"); 
	std::ifstream inF(qPrintable(fileName), std::ios::in);

	int num;
	inF >> num;
	activeScene->suggestions.clear();
	for (int i=0; i<num; i++)
	{
		EditSuggestion sgt;
		inF >> sgt.center >> sgt.direction >> sgt.value;
		activeScene->suggestions.push_back(sgt);
	}

	inF.close();
}

void StackerPanel::setBBTolerance( double tol )
{
	BB_TOLERANCE = tol;
}

void StackerPanel::setNumExpectedSolutions( int num )
{
	NUM_EXPECTED_SOLUTION = num;
}

void StackerPanel::setTargetStackability( double s )
{
	TARGET_STACKABILITY = s;
}

void StackerPanel::onNomalizeMeshChecked( )
{
	NORMALIZE_MESH = panel.normalzieMesh->isChecked();
}

void StackerPanel::onMoveCenterToOriginChecked()
{
	MOVE_CENTER_TO_ORIGIN = panel.moveCenterToOrigin->isChecked();
}