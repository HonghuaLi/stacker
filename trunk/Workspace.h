#pragma once

#include <QtGui/QMainWindow>
#include "ui_Workspace.h"

#include <QMdiSubWindow>
#include <QFileDialog>

#include "Scene.h"

#include "StackerPanel.h"
#include "WiresPanel.h"
#include "DeformerPanel.h"

extern QStringList main_args;
extern QMap<QString, QSegMesh> all_objects;

class Workspace : public QMainWindow
{
	Q_OBJECT

public:
	Workspace(QWidget *parent = 0, Qt::WFlags flags = 0);
	~Workspace();

public slots:
	void addNewScene();
	void importObject();
	void sceneFocusChanged(Scene* scene);

signals:
	void importedObject(QString fileName);

private:
	Ui::WorkspaceClass ui;

	StackerPanel * sp;
	WiresPanel * wp;
	DeformerPanel * dp;

	int sceneCount;
};
