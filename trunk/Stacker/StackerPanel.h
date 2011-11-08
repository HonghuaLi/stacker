#pragma once

#include "ui_StackerWidget.h"
#include "Scene.h"
#include "StackerPreview.h"
#include "HiddenViewer.h"
#include "Offset.h"

class StackerPanel : public QWidget
{
	Q_OBJECT

public:
	StackerPanel();

private:
	Ui::StackerWidget panel;
	Scene * activeScene;
	StackerPreview * stacker_preview;

public:
	HiddenViewer * hidden_viewer;
	Offset * activeOffset;

public slots:
	void onOffsetButtonClicked();
	void setActiveScene(Scene *);
	void updateActiveObject();

signals:
	void activeSceneChanged();
};
