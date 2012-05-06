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
#include "Improver.h"
#include "Numeric.h"
#include "Offset.h"


StackerPanel::StackerPanel()
{
	// Object changes
	connect(this, SIGNAL(objectModified()), SLOT(updateActiveObject()));

	// The UI
	panel.setupUi(this);

	// Add a stacking preview widget
	previewer = new Previewer(this);
	QDockWidget * previewDock = new QDockWidget("Previewer");
	previewDock->setWidget (previewer);
	QGridLayout * previewLayout = (QGridLayout *) panel.previewBox->layout();
	previewLayout->addWidget(previewDock, previewLayout->rowCount() + 1, 0,1,6);
	previewer->setMinimumHeight(50);
	connect(panel.stackCount, SIGNAL(valueChanged(int)), previewer, SLOT(setStackCount(int)) );

	// Add a stacking hidden viewer widget for offset calculator
	hiddenViewer = new HiddenViewer();
	QDockWidget * hiddenDock = new QDockWidget("HiddenViewer");
	hiddenDock->setWidget (hiddenViewer);
	QGridLayout * hiddenLayout = (QGridLayout *) panel.hiddenviewBox->layout();
	hiddenLayout->addWidget(hiddenDock, hiddenLayout->rowCount() + 1, 0,1,3);
	hiddenDock->setFloating(true);
	hiddenDock->setWindowOpacity(1.0);
	int x = qApp->desktop()->availableGeometry().width();
	hiddenDock->move(QPoint(x - hiddenDock->width(),0)); //Move the hidden dock to the top right conner
	connect(panel.hidderViewerResolution, SIGNAL(valueChanged(int)), hiddenViewer, SLOT(setResolution(int)));

	// Offset function calculator
	activeOffset = new Offset(hiddenViewer);

	// Improve and suggest
	connect(panel.showPaths, SIGNAL(stateChanged(int)), SLOT(updateActiveObject()));
	connect(panel.improveButton, SIGNAL(clicked()), SLOT(onImproveButtonClicked()));
	improver = new Improver(activeOffset);
	connect(panel.targetS, SIGNAL(valueChanged(double)), improver, SLOT(setTargetStackability(double)));
	connect(panel.BBTolerance, SIGNAL(valueChanged(double)), improver, SLOT(setBBTolerance(double)) );
	connect(panel.numExpectedSolutions, SIGNAL(valueChanged(int)), improver, SLOT(setNumExpectedSolutions(int)) );
	connect(panel.localRadius, SIGNAL(valueChanged(int)), improver, SLOT(setLocalRadius(int)) );
	
	// Stacking direction
	connect(panel.searchDirectionButton, SIGNAL(clicked()), SLOT(searchDirection()));
			
	// Debugging
	connect(panel.hotspotsButton, SIGNAL(clicked()), SLOT(onHotspotsButtonClicked()));
	connect(panel.outputButton, SIGNAL(clicked()), SLOT(outputForPaper()));

	// Default values
	panel.numExpectedSolutions->setValue(improver->NUM_EXPECTED_SOLUTION);
	panel.BBTolerance->setValue(improver->BB_TOLERANCE);
	panel.targetS->setValue(improver->TARGET_STACKABILITY);
	panel.localRadius->setValue(improver->LOCAL_RADIUS);
	panel.hidderViewerResolution->setValue(hiddenViewer->height());
	panel.stackCount->setValue(previewer->stackCount);
}

StackerPanel::~StackerPanel()
{
	delete previewer;
	delete hiddenViewer;
	delete activeOffset;
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

void StackerPanel::setActiveScene( Scene * newScene )
{
	if(activeScene != newScene)
	{
		activeScene = newScene;
		previewer->setActiveObject(newScene->activeObject());
		hiddenViewer->setActiveObject(newScene->activeObject());
	}
}

void StackerPanel::updateActiveObject()
{
	// Offset
	if (panel.rotAroundAxis->isChecked())
		activeOffset->computeOffsetOfShape( ROT_AROUND_AXIS, panel.searchDensity->value() );
	else if (panel.rotFreeForm->isChecked())
		activeOffset->computeOffsetOfShape( ROT_FREE_FORM, panel.searchDensity->value() );
	else
		activeOffset->computeOffsetOfShape();

	// Preview
	previewer->updateActiveObject();
}

QSegMesh* StackerPanel::activeObject()
{
	if (activeScene)
		return activeScene->activeObject();
	else 
		return NULL;
}

void StackerPanel::print( QString message )
{
	printMessage(message);
}

void StackerPanel::draw()
{
	// Draw all suggestions
	if (panel.showPaths)
	{
		foreach(EditPath path, suggestions)
			path.draw();
	}

	// Draw the path of selected item

}

void StackerPanel::setActiveObject()
{
	// Set active object for hidden viewer and previewer
	previewer->setActiveObject(activeObject());
	hiddenViewer->setActiveObject(activeObject());


	Controller * ctrl = (Controller *) activeScene->activeObject()->ptr["controller"];

	// Update
	updateActiveObject();

	// Initialize solution tree
	treeNodes.clear();
	treeNodes["0"] = ctrl->getShapeState();

	panel.solutionTree->clear();
	QTreeWidgetItem * node = new QTreeWidgetItem(panel.solutionTree);
	node->setText(0, "0");
}

void StackerPanel::showMessage( QString message )
{
	emit(printMessage(message));
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
	previewer->saveStackObj(exportDir + "/" + data["stackPreview"], panel.stackCount->value(), 1.0);


	// 3) Save stuck points / region
	data["upperStuck"] = "upperStuck.dat";
	data["lowerStuck"] = "lowerStuck.dat";
	activeOffset->detectHotspots();
	double sampleSize = 1.0; // % 5
	activeOffset->saveHotSpots(exportDir + "/" + data["upperStuck"], 1, sampleSize);
	activeOffset->saveHotSpots(exportDir + "/" + data["lowerStuck"], -1, sampleSize);

	// 4) Save stacking direction
	Vec3d direction = previewer->stackDirection;
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
	int angleNum = 1;
	int num2 = num;		// maybe num2 = num * 2 for bias
	double r = 1.0;

	double x,y,z, theta = 0, phi = 0;

	double deltaTheta = M_PI / num;
	double deltaPhi = 2.0 * M_PI / num2;
	
	std::vector<Vec3d> samples;

	if(panel.rotAroundAxis->isChecked())
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

	if(panel.rotAroundAxis->isChecked()){
		for(int i = 0; i < samples.size(); i++){
			switch(panel.searchAxisID->value())
			{
			case 0: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(0,0,1)); break;
			case 1: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(0,1,0)); break;
			case 2: ROTATE_VEC(samples[i], M_PI / 2.0, Vec3d(1,0,0)); break;
			}

			activeObject()->getSegment(0)->debug_points.push_back(samples[i]);
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


QString StackerPanel::getItemId(QTreeWidgetItem* item)
{
	QString txt = item->text(0);

	return txt.replace(0,txt.lastIndexOf(":") + 1,"");
}

void StackerPanel::addChildren( QTreeWidgetItem* parent, QVector<ShapeState> &children )
{
	// Remove all children
	parent->takeChildren();

	// Add new children
	int id = treeNodes.size();
	foreach (ShapeState cs, children)
	{
		// Model
		QString stringID = QString("%1").arg(id);
		treeNodes[stringID] = cs;

		// View
		QTreeWidgetItem * child = new QTreeWidgetItem (parent);
		child->setText(0, stringID);
		parent->addChild(child);

		id++;
	}
}


void StackerPanel::onImproveButtonClicked()
{
	// Preconditions
	if (!activeScene)  return;
	if (activeScene->isEmpty()) {
		showMessage("There is no valid object.");
		return;
	}

	// The selected item (parent)
	QList<QTreeWidgetItem*> selectedItems = panel.solutionTree->selectedItems();
	if (selectedItems.isEmpty())
	{
		showMessage("No item is selected!");
		return;
	}

	// Execute
	Improver improver(activeOffset);
	int level = panel.isSuggesting ? panel.suggestLevels->value() : IMPROVER_MAGIC_NUMBER;
	improver.executeImprove(level);

	// Get children shape states
	QVector<ShapeState> childrenStates = improver.solutions;

	if (panel.isSuggesting)
	{
		while (!improver.candidateSolutions.empty())
		{
			childrenStates.push_back(improver.candidateSolutions.top());
			improver.candidateSolutions.pop();
		}
	}

	// Update the solution tree
	addChildren(selectedItems.first(), childrenStates);
}