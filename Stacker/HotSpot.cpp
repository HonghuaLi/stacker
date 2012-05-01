#include "HotSpot.h"

#include <iostream>
#include "MathLibrary/Bounding/MinOBB3.h"
#include "Primitive.h"
#include "Cuboid.h"
#include "Controller.h"

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
	//double occupancy  = hotPixels.size() / box.area();
	//if (occupancy < 0.4)
	if (!hotPixels.contains(box.Center))
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


void HotSpot::computeRepresentative(Controller* ctrl)
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
		{
			Point p1 = box.Center + box.Extent[2] * box.Axis[2];
			Point p2 = box.Center - box.Extent[2] * box.Axis[2];
			if (ctrl->getPrimitive(segmentID)->primType == CUBOID)
			{
				Cuboid* cuboid = (Cuboid*) ctrl->getPrimitive(segmentID);
				Vec3d vec = p2 - p1;
				Vec3d axis = cuboid->currBox.ClosestAxis(vec);
				p2 = p1 + dot(vec, axis) * axis;
			}
			rep.push_back(p1);
			rep.push_back(p2);
		}
		break;
	}
}
