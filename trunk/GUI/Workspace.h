#pragma once

#include <QtGui/QMainWindow>
#include "ui_Workspace.h"

#include <QMdiSubWindow>

#include "QMeshDoc.h"
#include "GUI/Scene.h"
#include "GUI/Tools/TransformationPanel.h"
#include "GUI/Tools/MeshInfoPanel.h"

// Stacker widgets
#include "Stacker/StackerPanel.h"
#include "Stacker/GroupPanel.h"
#include "Stacker/ControllerPanel.h"
#include "MathLibrary/Deformer/DeformerPanel.h"
#include "MathLibrary/Deformer/QVoxelDeformerPanel.h"

class Workspace : public QMainWindow
{
	Q_OBJECT

public:
	Workspace(QWidget *parent = 0, Qt::WFlags flags = 0);
	~Workspace();

	Ui::WorkspaceClass ui;
	Scene * activeScene;

public slots:
	void addNewScene();
	void setActiveScene(Scene* scene);
	void disconnectScene(Scene* scene);
	void sceneClosed(Scene* scene);
	void processUserStudyResults();
	void processUserStudyResults2();

private:
	MeshInfoPanel * mi;
	TransformationPanel * tp;
	
	// Stacker widgets
	StackerPanel		* sp;
	GroupPanel			* gp;
	ControllerPanel		* cp;
	DeformerPanel		* dp;
	QVoxelDeformerPanel * vdp;

	int sceneCount;
};
