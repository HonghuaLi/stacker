#pragma once;

#include <QMap>
#include <QVector>
#include <QString>
#include <queue>
#include "Stacker/EditPath.h"

class Group;

class ShapeState
{
public:
	// Geometry
	QMap< QString, void* > primStates;

	// Groups
	QMap<QString, Group*> groups;

	// Stacking
	Vec3d stacking_shift;
	double stackability;

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
