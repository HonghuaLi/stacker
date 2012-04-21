#pragma once

#include "Group.h"
#include "GraphicsLibrary/Basic/Plane.h"

class CoplanarGroup : public Group{

public:
	CoplanarGroup(GroupType newType) : Group(newType){}

	void process(QVector< Primitive* > segments);
	void regroup();
	void draw();

	Plane groupAxis[3];
};
