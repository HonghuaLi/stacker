#pragma once

#include <QVector>
#include <QString>

#include "ShapeState.h"
#include "ConstraintGraph.h"

class Controller;


class Propagator
{
public:
	Propagator( Controller* ctrl );

	// Regroup pair
	void regroupPair( QString id1, QString id2, bool sliding = true );
	// Sliding
	void slide(QString id);

	// Propagation
	void execute();

	// Propagate to \target
	void propagateTo( QString target );
	void solvePointJointConstraints( QString target, QVector<ConstraintGraph::Edge> &constraints );
private:
	Controller *		mCtrl;
	ConstraintGraph *	mGraph;
};