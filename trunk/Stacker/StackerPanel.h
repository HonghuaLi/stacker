#pragma once

#include "ui_StackerPanel.h"


class StackerPanel : public QWidget
{
	Q_OBJECT

private:
	Ui::StackerPanel panel;

public:
	StackerPanel();

public slots:
	void onStackButtonClicked();

signals:
	void StackButtonClicked();
};
