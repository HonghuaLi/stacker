#pragma once

#include "ui_StackerWidget.h"
#include "Scene.h"
#include "StackerPreview.h"

class StackerPanel : public QWidget
{
	Q_OBJECT

private:
	Ui::StackerWidget panel;
	Scene * activeScene;

public:
	StackerPanel();

	StackerPreview * stacker_preview;

public slots:
	void onOffsetButtonClicked();
	void setActiveScene(Scene *);
	void updateActiveObject();

signals:
	void activeSceneChanged();
};
