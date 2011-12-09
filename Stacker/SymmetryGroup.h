#pragma once

#include "Group.h"

class SymmetryGroup : public Group{

public:
	SymmetryGroup(Controller * controller, GroupType newType) : Group(controller, newType){}

	void process(std::vector<int> segments);

	void draw();
};
