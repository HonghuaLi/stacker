#pragma once

#include "ui_GroupWidget.h"
#include "Scene.h"

class GroupPanel : public QWidget{
	Q_OBJECT

public:
	GroupPanel(QWidget * parent = NULL);

	Ui::GroupManagmentWidget groupWidget;
	Scene * activeScene;

	QVector<QString> groupTypes;

	// Utility function
	int getItemId(QTreeWidgetItem* item);

public slots:
	void setActiveScene( Scene * newScene );
	void updateWidget();
	void removeSelectedItem();

signals:

};
