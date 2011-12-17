#pragma once

#include "Group.h"
#include "Plane.h"

class SelfSymmetryOne : public Group{

public:
	SelfSymmetryOne(Controller * controller, GroupType newType) : Group(controller, newType){}

	virtual void process(QVector< QString > segments);
	virtual void draw();
	virtual void save(std::ofstream &outF);
	virtual void load(std::ifstream &inF);

	int selectedPartId;
};
