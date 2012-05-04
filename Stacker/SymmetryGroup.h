#pragma once

#include "Group.h"
#include "GraphicsLibrary/Basic/Plane.h"

class SymmetryGroup : public Group{

public:
	SymmetryGroup(GroupType newType) : Group(newType){}

	// Inherited methods
	void process(QVector< Primitive* > segments);
	void regroup();
	void draw();

public:
	Plane symmetryPlane;
	QMap< QString, QVector<int> > correspondence;
};
