#pragma once

#include <QtGui/QMainWindow>
#include "ui_Workspace.h"

#include <QMdiSubWindow>
#include <QFileDialog>

#include "QMeshDoc.h"
#include "Scene.h"
#include "StackerPanel.h"
#include "WiresPanel.h"
#include "DeformerPanel.h"


class Workspace : public QMainWindow
{
	Q_OBJECT

public:
	Workspace(QWidget *parent = 0, Qt::WFlags flags = 0);
	~Workspace();

public slots:
	void addNewScene();
	void importObject();
	void setActiveScene(Scene* scene);

signals:
	void importObject(QString fileName);

private:
	Ui::WorkspaceClass ui;
	StackerPanel * sp;
	WiresPanel * wp;
	DeformerPanel * dp;

	QMeshDoc * mDoc;
	int sceneCount;
};