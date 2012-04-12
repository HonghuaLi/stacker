#pragma once

#include "ui_StackerWidget.h"
#include "GUI/Scene.h"
#include "StackerPreview.h"
#include "HiddenViewer.h"
#include "Controller.h"
#include "QDeformController.h"

// Forward declaration
class Offset;
class StackabilityImprover;

class StackerPanel : public QWidget
{
	Q_OBJECT

public:
	StackerPanel();

	// Active Object
	QSegMesh* activeObject();

	// Message
	void showMessage( QString message );

	Ui::StackerWidget panel;
	Scene * activeScene;
	StackerPreview * stacker_preview;
	HiddenViewer * hidden_viewer;

	// Core components
	Offset * activeOffset;
	StackabilityImprover * improver;

public slots:
	// Scene management
	void setActiveScene( Scene * );
	void updateActiveObject();

	// Buttons
	void onHotspotsButtonClicked();
	void onImproveButtonClicked();
	void onSuggestButtonClicked();
	void onSaveSuggestionsButtonClicked();
	void onLoadSuggestionsButtonClicked();
	void searchDirection();
	void outputForPaper();

	// Primitives modification
	void convertGC();
	void convertCuboid();

	// Joints
	void findJoints();
	void findPairwiseJoints();
	void setJointThreshold(double threshold);
	void setSkeletonJoints(int num);

	// Parameters
	void setStackCount(int num);

	// Solutions
	void setTargetStackability(double s);
	void setBBTolerance(double tol);
	void setNumExpectedSolutions(int num);
	void setSolutionID(int id);
	void setSuggestionID(int id);

signals:
	void printMessage( QString );
	void objectModified();
};
