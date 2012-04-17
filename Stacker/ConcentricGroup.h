#pragma once

#include "Group.h"

class ConcentricGroup : public Group{

public:
	ConcentricGroup( GroupType newType) : Group(newType){}

	void process(QVector< Primitive* > segments);
	QVector<QString> regroup();
	void draw();

	Vec3d axis;
	Vec3d center;
};
