#pragma once

#include "ui_GroupWidget.h"
#include "GUI/Scene.h"

class JointDetector;

class GroupPanel : public QWidget{
	Q_OBJECT

public:
	GroupPanel(QWidget * parent = NULL);

	// Types
	QVector<QString> groupTypes;

	// Joint detector
	JointDetector * jointDetector;

private:
	Ui::GroupManagmentWidget groupWidget;
	Scene * activeScene;
	QSegMesh* activeObject();
	QString getItemId(QTreeWidgetItem* item);

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
