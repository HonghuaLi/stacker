#pragma once

#include "ui_StackerWidget.h"
#include "Scene.h"
#include "StackerPreview.h"
#include "HiddenViewer.h"
#include "Offset.h"
#include "Contoller.h"
#include "ui_SpaceExplorer.h"

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
	double sumEnergy(std::vector< cuboidDeformParam > &params);

	Ui::StackerWidget panel;
	Scene * activeScene;
	StackerPreview * stacker_preview;
	HiddenViewer * hidden_viewer;
	Offset * activeOffset;
	Ui::ControllerDeformerWidget ctrlDeformer;

	Controller::Stat originalStats;

public slots:
	// Buttons
	void onOffsetButtonClicked();
	void onControllerButtonClicked();
	void onImproveButtonClicked();
	void onHotspotsButtonClicked();

	// Others
	void setActiveScene( Scene * );
	void updateActiveObject();

	void setConvexHullPrecision(int p);

	// Primitives modification
	void convertGC();
	void updateController();
	void resetCtrlDeformerPanel();

signals:
	void printMessage( QString );
	void objectModified();

};
