#include "HotSpot.h"

#include <iostream>
#include "MathLibrary/Bounding/OBB_PCA.h"
#include "MathLibrary/Bounding/Box3.h"
#include "Primitive.h"
#include "Cuboid.h"
#include "Controller.h"
#include "Utility/Macros.h"
#include "Numeric.h"

void HotSpot::print()
{
	QStringList types;
	types << "POINT" << "LINE" << "RING";

	std::cout << "side="   << side << '\t'
		<< "segmentID="	  << qPrintable(segmentID) << '\t'
		<< "defineHeight=" << defineHeight << '\t'
		<< "type=" << qPrintable(types[type]) << std::endl; 
}

void HotSpot::decideType(Controller* ctrl)
{
	// Special case for GC
	Primitive* prim = ctrl->getPrimitive(segmentID);
	if(prim->primType == GCYLINDER && prim->symmPlanes.size() == 2)
	{
		type = RING_HOTSPOT;
		return;
	}

	// Using STL and double
	std::vector<Vec3d> points;
	Vec2i center(0);
	foreach(Vec2i v, hotPixels)
	{
		points.push_back(Vec3d(v.x(), v.y(), 0));
		center += v;
	}
	center /= hotPixels.size();

	// Compute bounding box in 2D
	OBB_PCA obb;
	obb.build_from_points(points);
	Box3 box;
	box.Center = obb.center();
	box.Axis = obb.axis();
	box.Extent = obb.extents();
	box.sort();

	double ratio = box.Extent[2] / box.Extent[1];

	if (ratio > 4)
		type = LINE_HOTSPOT;
	else
	{
		for (int i = center.x() - 2; i < center.x() + 2; i++ ){
			for (int j = center.y() - 2; j < center.y() + 2; j++ ){

				if (hotPixels.contains(Vec2i(i, j)))
				{
					type = POINT_HOTSPOT;
					return;
				}
			}
		}

		type = RING_HOTSPOT;
	}


	if(type == LINE_HOTSPOT)
	{
		// Check the distribution in 3D
		// Line hot spot shouldn't be too short
		std::vector<Point> points;
		foreach(Point p, hotSamples)
			points.push_back(p);
		Point p1, p2;
		twoFurthestPoints(points, p1, p2);
		if( (p1-p2).norm() < 0.3)
			type = POINT_HOTSPOT;
	}
}


void HotSpot::computeRepresentative()
{
	if (type == RING_HOTSPOT || hotSamples.size() < 2) return;
	
	// Eliminate outliers in hot samples(may caused by un-projection from 2D to 3D)
	// Weak assumption: all hot spots are flat
	double mean_z = 0;
	foreach(Point p, hotSamples) mean_z += p.z();
	mean_z /= hotSamples.size();

	std::vector<Point> points;
	double tol = 0.01;

	while(points.size() < 3)
	{
		points.clear();

		foreach(Point p, hotSamples)
		{
			if ( abs(p.z() - mean_z) < tol )
				points.push_back(p);
		}

		tol += 0.05;
	}

	// The Representatives
	rep.clear();
	switch (type)
	{
	case POINT_HOTSPOT:
		rep.push_back(centroid(points));
		break;

	case LINE_HOTSPOT:
		{
			// Get the two furthest points
			Point p1, p2;
			twoFurthestPoints(points, p1, p2);
			rep.push_back(p1);
			rep.push_back(p2);
		}
		break;
	}
}
