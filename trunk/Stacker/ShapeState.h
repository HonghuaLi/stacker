#pragma once;

#include <QMap>
#include <QVector>
#include <QString>
#include <queue>



struct PrimitiveState
{
	 void* geometry;
	 bool isFrozen;
};


class ShapeState
{
public:
	QMap< QString, PrimitiveState > primStates;
	double deltaStackability;
	double distortion;

	QVector<ShapeState> history;

	double energy();

	// 
	QVector<QString> seeds;
};

struct lessDistortion
{
	bool operator () (ShapeState a, ShapeState b);
};

struct lessEnergy
{
	bool operator () (ShapeState a, ShapeState b);
};

typedef std::priority_queue< ShapeState, QVector<ShapeState>, lessEnergy >		PQShapeShateLessEnergy;
typedef std::priority_queue< ShapeState, QVector<ShapeState>, lessDistortion >	PQShapeShateLessDistortion;
