#pragma once

#include "Group.h"

class ConcentricGroup : public Group{

public:
	ConcentricGroup( GroupType newType) : Group(newType){}

	void process(QVector< Primitive* > segments);
	void regroup();
	void draw();

	Vec3d axis;
	Vec3d center;
};
