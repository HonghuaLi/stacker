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


struct ShapeState
{
	QMap< QString, PrimitiveState > primStates;
	double deltaStackability;
	double distortion;

	QVector<ShapeState> history;

	double energy()
	{
		double alpha = 0.7;
		return  deltaStackability * alpha - distortion * (1-alpha);
	}

	// 
	QVector<QString> seeds;
};

struct lessDistortion
{
	bool operator () (ShapeState a, ShapeState b)
	{
		return a.distortion < b.distortion;
	}
};

struct lessEnergy
{
	bool operator () (ShapeState a, ShapeState b)
	{
		return a.energy() < b.energy();
	}
};

typedef std::priority_queue< ShapeState, QVector<ShapeState>, lessEnergy >		PQShapeShateLessEnergy;
typedef std::priority_queue< ShapeState, QVector<ShapeState>, lessDistortion >	PQShapeShateLessDistortion;
