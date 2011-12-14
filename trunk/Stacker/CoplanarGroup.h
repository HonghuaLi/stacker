#pragma once

#include "Group.h"

class CoplanarGroup : public Group{

public:
	CoplanarGroup(Controller * controller, GroupType newType) : Group(controller, newType){}

	void process(QVector< QString > segments);

	void draw();

};
