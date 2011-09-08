#pragma once

#include "ui_StackerWidget.h"

using namespace Ui;

class StackerPanel : public QWidget
{
	Q_OBJECT

private:
	StackerWidget sw;

public:
	StackerPanel();

public slots:
	void stackButtonClicked();

signals:
	void doStuffScene(QString);
};
