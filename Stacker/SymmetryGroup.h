#pragma once

#include "Group.h"
#include "Plane.h"

class SymmetryGroup : public Group{

public:
	SymmetryGroup(Controller * controller, GroupType newType) : Group(controller, newType){}

	void process(QVector< QString > segments);
	QVector<Primitive *> regroup();
	void draw();

	Plane symmetryPlane;
	std::vector<Point> allCenters;

	// Symmetry axis for grater than 4?
	Vec3d symmetryAxis;
	Vec3d symmetryAxisPos;
	bool hasSymmetryAxis;
};
