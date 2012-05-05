#pragma once;

#include <QMap>
#include <QVector>
#include <QString>
#include <queue>
#include "Stacker/EditPath.h"

class ShapeState
{
public:
	QMap< QString, void* > primStates;

	// Energy
	double deltaStackability;
	double distortion;
	double energy();

	// Editing path from parent
	EditPath path;

};

struct lessDistortion
{
	bool operator () (ShapeState a, ShapeState b);
};

struct lessEnergy
{
	bool operator () (ShapeState a, ShapeState b);
};

typedef std::priority_queue< ShapeState, QVector<ShapeState>, lessEnergy >		PQShapeStateLessEnergy;
typedef std::priority_queue< ShapeState, QVector<ShapeState>, lessDistortion >	PQShapeStateLessDistortion;
