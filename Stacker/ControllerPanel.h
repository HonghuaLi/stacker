#pragma once

#include "ui_ControllerWidget.h"
#include "GUI/Scene.h"

class Controller;

class ControllerPanel : public QWidget{
	Q_OBJECT

public:
	ControllerPanel(QWidget * parent = NULL);


	Ui::ControllerManagmentWidget controllerWidget;
	Scene * activeScene;


public slots:
	// Secen management
	void setActiveScene( Scene * newScene );

	// Update controller
	void updateController();

	// Primitives modification
	void setSkeletonJoints( int num );
	void convertGC();
	void convertCuboid();

	// Display
	void removeSelected();
	void clear();
	void togglePrimDisplay(int state);

	// Save and load
	void save();
	void load();

signals:

private:
	QSegMesh* activeObject();
	Controller * controller();

};
