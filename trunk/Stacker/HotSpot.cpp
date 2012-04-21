#include "HotSpot.h"

#include <iostream>
#include "MathLibrary/Bounding/MinOBB3.h"

void HotSpot::print()
{
	std::cout << "side="   << side << '\t'
		<< "segmentID="	  << qPrintable(segmentID) << '\t'
		<< "defineHeight=" << defineHeight << '\t'
		<< "type=" << type << std::endl; 
}

void HotSpot::decideType()
{
	// Using STL and double
	std::vector<Vec2d> pixels;
	foreach(Vec2i v, hotPixels)
		pixels.push_back(Vec2d(v.x(), v.y()));

	// Compute bounding box in 2D
	MinOBB2 obb(pixels);
	MinOBB2::Box2 box = obb.getBox2();

	// Check if it is a ring
	double occupancy  = hotPixels.size() / box.area();
	if (occupancy < 0.4)
		type = RING_HOTSPOT;
	else
	{
		double ratio = box.Extent[0] / box.Extent[1];
		if (ratio < 1) ratio = 1/ratio;

		if (ratio > 4)
			type = LINE_HOTSPOT;
		else
			type = POINT_HOTSPOT;
	}
}


void HotSpot::computeRepresentative()
{
	if (type == RING_HOTSPOT) return;
	
	// Use STL
	std::vector<Point> points;
	foreach(Point p, hotSamples)
		points.push_back(p);

	// Eliminate outliers in hot samples(may caused by un-projection from 2D to 3D)
	// .....

	// Compute minimal bounding box in 3D
	MinOBB3 obb(points);
	Box3 box = obb.mMinBox;
	box.sort();

	rep.clear();
	switch (type)
	{
	case POINT_HOTSPOT:
		rep.push_back(box.Center);
		break;
	case LINE_HOTSPOT:
		rep.push_back(box.Center + box.Extent[2] * box.Axis[2]);
		rep.push_back(box.Center - box.Extent[2] * box.Axis[2]);
		break;
	}
}
