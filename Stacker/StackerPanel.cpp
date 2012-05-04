#include "StackerPanel.h"
#include "Controller.h"
#include "Cuboid.h"
#include "MathLibrary/Bounding/ConvexHull3.h"
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
#include "StackerGlobal.h"
#include "Numeric.h"
#include "StackabilityImprover.h"


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
	layout->addWidget(previewDock, row++, 0,1,6);
	stacker_preview->setMinimumHeight(50);

	// Add a stacking hidden viewer widget for offset calculator
	hidden_viewer = new HiddenViewer();
	QDockWidget * hiddenDock = new QDockWidget("Hidden");
	hiddenDock->setWidget (hidden_viewer);
	layout->addWidget(hiddenDock, row++, 0,1,3);
	hiddenDock->setFloating(true);
	hiddenDock->setWindowOpacity(1.0);
	int x = qApp->desktop()->availableGeometry().width();
	hiddenDock->move(QPoint(x - hiddenDock->width(),0)); //Move the hidden dock to the top right conner
	
	// Offset function calculator
	activeOffset = new Offset(hidden_viewer);
	improver = new StackabilityImprover(activeOffset);

	// Scene manager
	connect(this, SIGNAL(objectModified()), SLOT(updateActiveObject()));
	connect(panel.stackCount, SIGNAL(valueChanged(int)), this, SLOT(setStackCount(int)) );
	connect(panel.hidderViewerSize, SIGNAL(valueChanged(int)), hidden_viewer, SLOT(setResolution(int)));

	// Hotspot
	connect(panel.hotspotsButton, SIGNAL(clicked()), SLOT(onHotspotsButtonClicked()));

	// Stacking direction
	connect(panel.searchDirectionButton, SIGNAL(clicked()), SLOT(searchDirection()));

	// Solution
	connect(panel.BBTolerance, SIGNAL(valueChanged(double)), this, SLOT(setBBTolerance(double)) );
	connect(panel.numExpectedSolutions, SIGNAL(valueChanged(int)), this, SLOT(setNumExpectedSolutions(int)) );
	connect(panel.targetS, SIGNAL(valueChanged(double)), this, SLOT(setTargetStackability(double)));
	connect(panel.improveButton, SIGNAL(clicked()), SLOT(onImproveButtonClicked()));
	connect(panel.suggestButton, SIGNAL(clicked()), SLOT(onSuggestButtonClicked()));
	connect(panel.solutionID, SIGNAL(valueChanged(int)), this, SLOT(setSolutionID(int)));
	
	// Suggestion
	connect(panel.suggestionID, SIGNAL(valueChanged(int)), this, SLOT(setSuggestionID(int)));
	connect(panel.saveSuggestionsButton, SIGNAL(clicked()), SLOT(onSaveSuggestionsButtonClicked()));
	connect(panel.loadSuggestionsButton, SIGNAL(clicked()), SLOT(onLoadSuggestionsButtonClicked()));
		
	// Paper stuff
	connect(panel.outputButton, SIGNAL(clicked()), SLOT(outputForPaper()));

	// Default values
	panel.numExpectedSolutions->setValue(NUM_EXPECTED_SOLUTION);
	panel.BBTolerance->setValue(BB_TOLERANCE);
	panel.targetS->setValue(TARGET_STACKABILITY);
	panel.hidderViewerSize->setValue(HIDDEN_VIEWER_SIZE);
}

void StackerPanel::onImproveButtonClicked()
{
	if (!activeScene || activeScene->isEmpty())	{
		showMessage("There is no valid object.");
		return;
	}

	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	if (!ctrl)	{
		showMessage("There is no controller built.");
		return;
	}

	improver->executeImprove();

	int total = improver->solutions.size();
	panel.numSolution->setText(QString("/ %1").arg(total));
	panel.suggestionID->setValue(0);

	emit(objectModified());
}

void StackerPanel::onHotspotsButtonClicked()
{
	if(!activeScene || !activeObject())
		return;

	activeOffset->detectHotspots();
	showMessage("Hot spots are detected.");

	activeOffset->showHotSpots();
	if(VBO::isVBOSupported()) emit(objectModified()); 
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
	// Offset
	if (panel.rotAroundAxis->isChecked())
		activeOffset->computeOffsetOfShape( ROT_AROUND_AXIS, panel.rotStackingDensity->value() );
	else if (panel.rotFreeForm->isChecked())
		activeOffset->computeOffsetOfShape( ROT_FREE_FORM, panel.rotStackingDensity->value() );
	else
		activeOffset->computeOffsetOfShape();

	// Preview
	stacker_preview->stackCount = panel.stackCount->value();
	stacker_preview->updateActiveObject();

	// Improver
	improver->clear();
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

void StackerPanel::setSolutionID(int id)
{
	int total = improver->solutions.size();
	panel.numSolution->setText(QString("/ %1").arg(total));

	id = (0==total)? 0 : (id % total);
	panel.solutionID->setValue(id);
	improver->showSolution(id);

	emit(objectModified());
}

void StackerPanel::setSuggestionID(int id)
{
	int total = improver->suggestSolutions.size();
	panel.numSuggestion->setText(QString("/ %1").arg(total));

	id = (0==total)? 0 : (id % total);
	panel.suggestionID->setValue(id);
	improver->showSuggestion(id);

	emit(objectModified());
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
	double maxUE = getMaxValue(activeOffset->upperEnvelope);
	double minLE = getMinValue(activeOffset->lowerEnvelope);

	saveAsImage(activeOffset->upperEnvelope, maxUE, exportDir + "/" + data["upperEnvelope"]);
	saveAsImage(activeOffset->lowerEnvelope, minLE, exportDir + "/" + data["lowerEnvelope"]);

	saveAsImage(activeOffset->offset, activeOffset->O_max, exportDir + "/" + data["offsetImg"]);
	saveAsData(activeOffset->offset, 1.0, exportDir + "/" + data["offsetData"]);
	data["O_max"] = QString::number(activeOffset->O_max);
	data["Stackability"] = QString::number(activeOffset->getStackability());

	// Save visualized 3D offset function:
	int w = activeOffset->offset.front().size();
	int h = activeOffset->offset.size();
	double q = 1.0 / Max(w,h);
	QStringList vertices, faces;

	for(int y = 0; y < h ; y++){
		for(int x = 0; x < w ; x++){
			double c = Max(0, activeOffset->offset[y][x]);
			vertices.push_back(QString("v %1 %2 %3").arg(x * q).arg(y * q).arg(c));

			if(y < h - 2 && x < w - 2){
				int col = x, row = y;
				int v1 = col + ((row) * w);
				int v2 = v1 + 1;
				int v3 = col + ((row + 1) * w);
				int v4 = v3 + 1;
				faces.push_back(QString("f %1 %2 %3 %4").arg(1+v1).arg(1+v2).arg(1+v4).arg(1+v3));
			}
		}
	}

	// Output
	std::ofstream outF(qPrintable(exportDir + "/offset_surface.obj"), std::ios::out);
	foreach(QString vertexLine, vertices) outF << qPrintable(vertexLine)  << std::endl;
	outF << "# num vertices " << vertices.size()  << "\n\n";
	foreach(QString faceLine, faces) outF << qPrintable(faceLine)  << std::endl;
	outF << "# num faces " << faces.size()  << "\n";
	outF.close();


	// 2) Save meshes stacked
	data["stackPreview"] = "stackPreview.obj";
	stacker_preview->saveStackObj(exportDir + "/" + data["stackPreview"], panel.stackCount->value(), 1.0);


	// 3) Save stuck points / region
	data["upperStuck"] = "upperStuck.dat";
	data["lowerStuck"] = "lowerStuck.dat";
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

	std::cout<< panel.searchAxis->value() << "\n";

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
			case 0: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(0,0,1)); break;
			case 1: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(0,1,0)); break;
			case 2: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(1,0,0)); break;
			}

			//activeObject()->getSegment(0)->debug_points.push_back(samples[i]);
		}
	}
	
	int sampleCount = 0;
	double maxStackbaility = activeOffset->getStackability(true); // don't go worse than start
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

			double stackability = activeOffset->getStackability(true);
			
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

	// Recompute normals and bounding box
	activeObject()->build_up();

	emit(objectModified());
}


void StackerPanel::setStackCount( int num )
{
	stacker_preview->stackCount = num;
	stacker_preview->updateActiveObject();
}

void StackerPanel::onSuggestButtonClicked()
{
	improver->suggestions.clear();

	isSuggesting = true;
	improver->executeImprove(0);
	isSuggesting = false;
	
	int total = improver->suggestSolutions.size();
	panel.numSuggestion->setText(QString("/ %1").arg(total));
	panel.suggestionID->setValue(0);
	emit(objectModified());
}

void StackerPanel::onSaveSuggestionsButtonClicked()
{
	if(!activeScene || !activeObject())	return;

	QVector< EditingSuggestion > &suggestions = suggestions;
	if (suggestions.empty()) return;

	QString fileName = QFileDialog::getSaveFileName(0, "Export Groups", "", "Group File (*.sgt)"); 
	std::ofstream outF(qPrintable(fileName), std::ios::out);

	outF << suggestions.size() << std::endl;
	foreach(EditingSuggestion sgt, suggestions)
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
	improver->suggestions.clear();
	for (int i=0; i<num; i++)
	{
		EditingSuggestion sgt;
		inF >> sgt.center >> sgt.direction >> sgt.value;
		improver->suggestions.push_back(sgt);
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

void StackerPanel::print( QString message )
{
	printMessage(message);
}
