#pragma once

#include "ui_ControllerWidget.h"
#include "Scene.h"

class ControllerPanel : public QWidget{
	Q_OBJECT

public:
	ControllerPanel(QWidget * parent = NULL);

	Ui::ControllerManagmentWidget controllerWidget;
	Scene * activeScene;

public slots:
	void setActiveScene( Scene * newScene );
	void removeSelected();
	void save();
	void load();
	void clear();

private:
	Controller * controller();

signals:

};
