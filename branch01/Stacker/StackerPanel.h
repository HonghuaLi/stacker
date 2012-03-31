#pragma once

#include "ui_StackerWidget.h"
#include "GUI/Scene.h"
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

	Ui::StackerWidget panel;
	Scene * activeScene;
	StackerPreview * stacker_preview;
	HiddenViewer * hidden_viewer;
	Offset * activeOffset;

public slots:
	// Buttons
	void onOffsetButtonClicked();
	void onImproveButtonClicked();
	void onHotspotsButtonClicked();
	void onSuggestButtonClicked();
	void onSaveSuggestionsButtonClicked();
	void onLoadSuggestionsButtonClicked();

	// Others
	void setActiveScene( Scene * );
	void updateActiveObject();

	void setConvexHullPrecision(int p);
	void setHotRange(double range);
	void setJointThreshold(double threshold);
	void setSkeletonJoints(int num);
	void setStackCount(int num);
	void setBBTolerance(double tol);
	void setNumExpectedSolutions(int num);
	void setSolutionID(int id);
	void setSuggestionID(int id);
	void setTargetStackability(double s);

	void onNomalizeMeshChecked();
	void onMoveCenterToOriginChecked();

	// Primitives modification
	void convertGC();
	void convertCuboid();

	// Joints
	void findJoints();
	void findPairwiseJoints();

	void searchDirection();

	void outputForPaper();

signals:
	void printMessage( QString );
	void objectModified();
};
