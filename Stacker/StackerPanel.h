#pragma once

#include "ui_StackerWidget.h"
#include "Scene.h"
#include "Stacker.h"

class StackerPanel : public QWidget
{
	Q_OBJECT

private:
	Ui::StackerWidget panel;
	Scene * activeScene;

public:
	StackerPanel();

public slots:
	void onOffsetButtonClicked();
	void setActiveScene(Scene *);
};
