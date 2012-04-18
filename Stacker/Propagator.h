#pragma once

#include <QVector>
#include <QString>

#include "ShapeState.h"

class Controller;

class Propagator
{
public:
	Propagator( Controller* ctrl );

	// Propagation
	void weakPropagate(QVector<QString> seeds);
	void weakPropagate();
	QVector<ShapeState> strongPropagate();
	void regroupPair(QString id1, QString id2);

private:
	Controller * mCtrl;
};