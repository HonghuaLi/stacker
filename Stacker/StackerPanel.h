#pragma once

#include "ui_StackerWidget.h"

class StackerPanel : public QWidget
{
	Q_OBJECT

private:
	Ui::StackerWidget panel;

public:
	StackerPanel();

public slots:
	void onStackButtonClicked();

signals:
	void StackButtonClicked();
};
