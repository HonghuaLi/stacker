#pragma once

#include "ui_StackerWidget.h"
#include "GUI/Scene.h"
#include "Previewer.h"
#include "HiddenViewer.h"
#include "Controller.h"
#include "QManualDeformer.h"

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
	Previewer * previewer;
	HiddenViewer * hiddenViewer;

	// Core components
	Offset * activeOffset;
	StackabilityImprover * improver;

public slots:
	// Scene management
	void setActiveScene( Scene * newScene);
	void updateActiveObject();

	// Improve and suggest
	void onImproveButtonClicked();
	void onSuggestButtonClicked();
	void searchDirection();
	void setTargetStackability(double s);
	void setBBTolerance(double tol);
	void setNumExpectedSolutions(int num);

	// Message
	void print(QString message);

	// Debug
	void onHotspotsButtonClicked();
	void outputForPaper();


signals:
	void printMessage( QString );
	void objectModified();
};
