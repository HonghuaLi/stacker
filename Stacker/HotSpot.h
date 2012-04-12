#pragma once

#include <vector>
#include <iostream>
#include <Qstring>
#include "GraphicsLibrary/Mesh/SurfaceMesh/Vector.h"

struct HotSpot
{
	int side;
	int hotRegionID;
	QString segmentID;
	bool defineHeight;
	bool isRing;
	std::vector< Vec3d > hotSamples;

	void print();
	Vec3d hotPoint();
};
