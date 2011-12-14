#pragma once

#include "Group.h"

class ConcentricGroup : public Group{

public:
	ConcentricGroup(Controller * controller, GroupType newType) : Group(controller, newType){}

	void process(QVector< QString > segments);

	void draw();

	Vec3d axis;
	Vec3d center;
};
