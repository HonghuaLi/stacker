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
	void onImproveButtonClicked();
	void onHotspotsButtonClicked();
	void onSolutionButtonClicked();

	// Others
	void setActiveScene( Scene * );
	void updateActiveObject();

	void setConvexHullPrecision(int p);
	void setHotRange(double range);
	void setJointThreshold(double threshold);
	void setSkeletonJoints(int num);

	// Primitives modification
	void convertGC();
	void convertCuboid();
	void updateController();
	void resetCtrlDeformerPanel();
	void selectModeController();
	void selectModeControllerElement();

	// Joints
	void findJoints();
	void findPairwiseJoints();

	void searchDirection();

	void outputForPaper();

signals:
	void printMessage( QString );
	void objectModified();
};
