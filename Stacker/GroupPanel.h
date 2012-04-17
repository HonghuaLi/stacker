#pragma once

#include "ui_GroupWidget.h"
#include "GUI/Scene.h"

class GroupPanel : public QWidget{
	Q_OBJECT

public:
	GroupPanel(QWidget * parent = NULL);

	Ui::GroupManagmentWidget groupWidget;
	Scene * activeScene;

	// Types
	QVector<QString> groupTypes;

	// Utility function
	QString getItemId(QTreeWidgetItem* item);

	// Active Object
	QSegMesh* activeObject();

public slots:
	// Sence management
	void setActiveScene( Scene * newScene );
	void updateWidget();

	// Joint detection
	void findJoints();
	void findPairwiseJoints();
	void setJointThreshold(double threshold);

	// Group tree
	void removeSelectedItem();
	void clearGroups();
	void toggleGroupDisplay(int state);

	// Load and save
	void saveGroups();
	void loadGroups();
};
