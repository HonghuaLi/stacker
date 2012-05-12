#include "HotSpot.h"

#include <iostream>
#include "MathLibrary/Bounding/OBB_PCA.h"
#include "Primitive.h"
#include "Cuboid.h"
#include "Controller.h"
#include "Macros.h"
#include "Numeric.h"

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
	std::vector<Vec3d> pixels;
	foreach(Vec2i v, hotPixels)
		pixels.push_back(Vec3d(v.x(), v.y(), 0));

	// Compute bounding box in 2D
	OBB_PCA obb;
	obb.build_from_points(pixels);

	Vec3d extent = obb.extents();
	QVector<double> tmp;
	for (int i = 0; i< 3; i++)
	{
		if (extent[i] > 2)
			tmp.push_back(extent[i]);
	}

	if (tmp.size() != 2)
	{
		std::cout << "NOT 2D !!\n";
		type = POINT_HOTSPOT;
		return;
	}

	double ratio = tmp[0] / tmp[1];
	if (ratio < 1) ratio = 1/ratio;

	if (ratio > 4)
		type = LINE_HOTSPOT;
	else
	{
		Vec2i center((int)obb.center().x(), (int)obb.center().y());
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
}


void HotSpot::computeRepresentative(Controller* ctrl)
{
	if (type == RING_HOTSPOT) return;
	

	// Eliminate outliers in hot samples(may caused by un-projection from 2D to 3D)
	// Weak assumption: all hot spots are flat
	double mean_z = 0;
	foreach(Point p, hotSamples) mean_z += p.z();
	mean_z /= hotSamples.size();

	std::vector<Point> points;
	foreach(Point p, hotSamples)
	{
		if ( abs(p.z() - mean_z) < 0.01 )
			points.push_back(p);
	}

	// Compute minimal bounding box in 3D
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
