#pragma once

#include <QtGui/QMainWindow>
#include "ui_Workspace.h"

#include <QMdiSubWindow>

#include "QMeshDoc.h"
#include "Scene.h"
#include "StackerPanel.h"
#include "WiresPanel.h"
#include "DeformerPanel.h"
#include "TransformationPanel.h"

class Workspace : public QMainWindow
{
	Q_OBJECT

public:
	Workspace(QWidget *parent = 0, Qt::WFlags flags = 0);
	~Workspace();

public slots:
	void addNewScene();
	void setActiveScene(Scene* scene);

private:
	Ui::WorkspaceClass ui;
	StackerPanel * sp;
	WiresPanel * wp;
	DeformerPanel * dp;
	TransformationPanel * tp;

	Scene * activeScene;

	QMeshDoc * mDoc;
	int sceneCount;
};
