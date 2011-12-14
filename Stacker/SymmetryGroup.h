#pragma once

#include "Group.h"
#include "Plane.h"

class SymmetryGroup : public Group{

public:
	SymmetryGroup(Controller * controller, GroupType newType) : Group(controller, newType){}

	void process(QVector< QString > segments);

	void draw();

	std::vector<Plane> symmetryPlanes;
};
