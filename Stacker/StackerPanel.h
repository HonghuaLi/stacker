#pragma once

#include "ui_StackerWidget.h"
#include "ui_SpaceExplorer.h"
#include "Scene.h"
#include "StackerPreview.h"
#include "HiddenViewer.h"
#include "Offset.h"
#include "Controller.h"
#include "QDeformController.h"

class StackerPanel : public QWidget
{
	Q_OBJECT

public:
	StackerPanel();

	// Active Object
	QSegMesh* activeObject();

	// Message
	void showMessage( QString message );

	// Optimization
	void gradientDescentOptimize();

	double sumEnergy( );
	Ui::StackerWidget panel;
	Ui::ControllerDeformerWidget ctrlDeformer;
	Scene * activeScene;
	StackerPreview * stacker_preview;
	HiddenViewer * hidden_viewer;
	Offset * activeOffset;

public slots:
	// Buttons
	void onOffsetButtonClicked();
	void onControllerButtonClicked();
	void onImproveButtonClicked();
	void onHotspotsButtonClicked();
	void onIterateButtonClicked();
	void onHotSolutionButtonClicked();

	// Others
	void setActiveScene( Scene * );
	void updateActiveObject();

	void setConvexHullPrecision(int p);
	void setHotspotFilterSize(int size);
	void setHotRange(double range);

	// Primitives modification
	void convertGC();
	void updateController();
	void resetCtrlDeformerPanel();
	void userControlledPrimatives();
	
	void findJoints();

	void outputForPaper();

signals:
	void printMessage( QString );
	void objectModified();

};
