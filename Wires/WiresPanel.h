#pragma once

#include "ui_WiresWidget.h"
using namespace Ui;

#include "Scene.h"

class WiresPanel : public QWidget
{
	Q_OBJECT

private:
	WiresWidget ww;
	Scene * activeScene;

public:
	WiresPanel();

public slots:
	void analyzeButtonClicked();
	void setActiveScene(Scene *);
	
signals:
	void wiresFound( QVector<Wire> );
};
