#pragma once

#include <QVector>
#include <QString>

#include "ShapeState.h"

class Controller;
class ConstraintGraph;

class Propagator
{
public:
	Propagator( Controller* ctrl );

	// Regroup pair
	void regroupPair(QString id1, QString id2);

	// Propagation
	void execute();

	// Solve constraints
	void solveConstraints(QString target, QVector<QString> constraints);

private:
	Controller *		mCtrl;
	ConstraintGraph *	mGraph;
};