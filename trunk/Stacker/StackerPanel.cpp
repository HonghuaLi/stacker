#include "StackerPanel.h"

#include <QDockWidget>
#include <QVBoxLayout>
#include <QDesktopWidget>
#include <QFile>
#include <QDir>

#include <fstream>
#include <algorithm>
#include <numeric>

#include "GUI/Scene.h"
#include "Macros.h"
#include "Improver.h"
#include "Previewer.h"
#include "HiddenViewer.h"
#include "Controller.h"
#include "Offset.h"


StackerPanel::StackerPanel()
{
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
	connect(panel.solutionTree, SIGNAL(itemSelectionChanged()), SLOT(setSelectedShapeState()));
	improver = new Improver(activeOffset);
	connect(panel.targetS, SIGNAL(valueChanged(double)), improver, SLOT(setTargetStackability(double)));
	connect(panel.BBTolerance, SIGNAL(valueChanged(double)), improver, SLOT(setBBTolerance(double)) );
	connect(panel.numExpectedSolutions, SIGNAL(valueChanged(int)), improver, SLOT(setNumExpectedSolutions(int)) );
	connect(panel.localRadius, SIGNAL(valueChanged(int)), improver, SLOT(setLocalRadius(int)) );
	
	// Stacking direction
	connect(panel.searchType, SIGNAL(valueChanged(int)), activeOffset, SLOT(setSearchType(int)));
	connect(panel.coneSize, SIGNAL(valueChanged(double)), activeOffset, SLOT(setConeSize(double)));
	connect(panel.searchDensity, SIGNAL(valueChanged(int)), activeOffset, SLOT(setSearchDensity(int)));
	panel.searchDensity->setValue(activeOffset->searchDensity);
			
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
	panel.searchType->setValue(activeOffset->searchType);
}

StackerPanel::~StackerPanel()
{
	delete previewer;
	delete hiddenViewer;
	delete activeOffset;
	delete improver;
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
		if (activeObject())
			setActiveObject();
	}
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

	// Offset
	activeOffset->computeStackability();

	// Preview
	previewer->updateActiveObject();

	resetSolutionTree();
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
	activeOffset->computeStackability();
	

	// 1) Save envelopes + offset function (both image + values)
	double maxUE = getMaxValue(activeOffset->upperEnvelope);
	double minLE = getMinValue(activeOffset->lowerEnvelope);

	saveAsImage(activeOffset->upperEnvelope,exportDir + "/" + data["upperEnvelope"]);
	saveAsImage(activeOffset->lowerEnvelope,exportDir + "/" + data["lowerEnvelope"]);

	saveAsImage(activeOffset->offset, exportDir + "/" + data["offsetImg"]);
	saveAsData(activeOffset->offset, 1.0, exportDir + "/" + data["offsetData"]);
	data["O_max"] = QString::number(activeOffset->O_max);
	data["Stackability"] = QString::number(activeObject()->val["stackability"]);

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
	Vec3d direction = activeObject()->vec["stacking_direction"];
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

	QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

	// Current selection
	QTreeWidgetItem * currItem = selectedItem();

	// Execute
	int level = panel.isSuggesting ? panel.suggestLevels->value() : IMPROVER_MAGIC_NUMBER;
	improver->execute(level);

	// Get children shape states
	QVector<ShapeState> childrenStates = improver->solutions;

	if (panel.isSuggesting)
	{
		while (!improver->candidateSolutions.empty())
		{
			childrenStates.push_back(improver->candidateSolutions.top());
			improver->candidateSolutions.pop();
		}
	}

	// Update the solution tree
	addChildren(currItem, childrenStates);

	QApplication::restoreOverrideCursor();
}

QTreeWidgetItem* StackerPanel::selectedItem()
{
	QTreeWidgetItem * item = NULL;

	QList<QTreeWidgetItem*> selectedItems = panel.solutionTree->selectedItems();
	if (!selectedItems.isEmpty()) item = selectedItems.first();

	return item;
}

void StackerPanel::setSelectedShapeState()
{
	QTreeWidgetItem * currItem = selectedItem();
	if (currItem && treeNodes.size()>1 )
	{
		QString currID = currItem->text(0);
		std::cout << "Current shape state id:" << qPrintable(currID) << std::endl;
		ShapeState currState = treeNodes[currID];		
		ctrl()->setShapeState(currState);

		previewer->updateActiveObject();
		emit(objectModified());
	}
}

Controller* StackerPanel::ctrl()
{
	if (activeObject())
		return (Controller*)activeObject()->ptr["controller"];
	else
		return NULL;
}

void StackerPanel::resetSolutionTree()
{
	treeNodes.clear();
	panel.solutionTree->clear();

	if (ctrl())
	{
		QString id("0");
		treeNodes[id] = ctrl()->getShapeState();

		QTreeWidgetItem * node = new QTreeWidgetItem(panel.solutionTree);
		node->setText(0, id);
		node->setSelected(true);
	}
}

void StackerPanel::updateActiveObject()
{
	activeOffset->computeStackability();
	previewer->updateActiveObject();
}

