#include "JointDetector.h"

#include "Primitive.h"
#include "Cuboid.h"
#include "PointJointGroup.h"
#include "LineJointGroup.h"

#include <algorithm>
#include "GraphicsLibrary/Voxel/Voxeler.h"
#include "MathLibrary/Bounding/MinOBB3.h"
#include "Numeric.h"

#define POINT_LINE_THRESHOLD 3
#define	LINE_2PONINTS_THRESHOLD 0.2

JointDetector::JointDetector()
{
	sliding = false;
	JOINT_THRESHOLD = 0.035;
}

QVector<Group*> JointDetector::detect( QVector<Primitive*> primitives )
{
	QVector<Group*> Joints;
	std::vector<Voxeler> voxels;

	foreach(Primitive * prim, primitives)
	{
		QSurfaceMesh mesh(prim->getGeometry());
		voxels.push_back( Voxeler(&mesh, JOINT_THRESHOLD) );
	}

	for(uint i = 0; i < primitives.size(); i++)
	{
		Primitive * a = primitives[i];

		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Primitive * b = primitives[j];

			std::vector<Voxel> intersection = voxels[i].Intersects(&voxels[j]);

			if(!intersection.empty())
			{
				// Voxel positions
				std::vector<Point> points;
				foreach( Voxel v, intersection){
					points.push_back(Point(v.x, v.y, v.z) * JOINT_THRESHOLD);
				}

				// Debug: visualize the intersection
				foreach (Point p, points)
					a->debugPoints.push_back(p);

				// Analyze the pair-wise intersection
				QVector<Group*> pairwiseJoints = analyzeIntersection(a, b, points);

				foreach(Group* g, pairwiseJoints)
					Joints.push_back(g);
			}

		}
	}

	return Joints;
}

QVector<Group*> JointDetector::analyzeIntersection( Primitive* a, Primitive* b, std::vector<Point> &intersection )
{
	QVector<Group*> Joints;

	// Segments
	QVector<Primitive*> segments;
	segments.push_back(a);
	segments.push_back(b);

	// Compute the minimal bounding box
	MinOBB3 obb(intersection);
	Box3 box = obb.mMinBox;
	box.sort();

	// Debug
	QSurfaceMesh * foo = a->getMesh();
	std::vector<Vec3d> line0,line1,line2;
	line0.push_back(box.Center);
	line0.push_back(box.Center + box.Extent[0] * box.Axis[0]);
	line1.push_back(box.Center);
	line1.push_back(box.Center + box.Extent[1] * box.Axis[1]);
	line2.push_back(box.Center);
	line2.push_back(box.Center + box.Extent[2] * box.Axis[2]);
	foo->debug_lines.push_back(line0);
	foo->debug_lines2.push_back(line1);
	foo->debug_lines3.push_back(line2);

	// Decide the type of joint
	std::cout << box.Extent[2]/box.Extent[1] << std::endl;
	
	if (box.Extent[2]/box.Extent[1] < POINT_LINE_THRESHOLD)
	{
		// Point-joint
		Joints.push_back( setupPointJointGroup(segments, intersection) );
	}
	else
	{
		Point p1 = box.Center + box.Extent[2] * box.Axis[2];
		Point p2 = box.Center - box.Extent[2] * box.Axis[2];

		// Two ends (projected to one major axes of the cuboid)
		Cuboid* cuboid = NULL;
		if (a->primType == CUBOID) cuboid = (Cuboid*) a;
		if (b->primType == CUBOID) cuboid = (Cuboid*) b;
		if (cuboid) 
		{
			Vec3d vec = p2 - p1;
			Vec3d axis = cuboid->currBox.ClosestAxis(vec);
			p2 = p1 + dot(vec, axis) * axis;
		}

		// Cluster points into two clusters
		std::vector< std::vector< Point > >clusters = twoClassClustering(intersection, p1, p2);

		// The distance between two clusters
		double dis = distanceCluster2Cluster(clusters[0], clusters[1]);

		std::cout << dis/box.Extent[2] << std::endl;

		if (dis/box.Extent[2] < LINE_2PONINTS_THRESHOLD)
		{
			// Line-joint
			LineJointGroup* LJG = new LineJointGroup(LINEJOINT);

			LJG->lineEnds.push_back(p1);
			LJG->lineEnds.push_back(p2);
			LJG->process(segments);

			Joints.push_back(LJG);
		}
		else
		{
			// Two point-joints
			Joints.push_back( setupPointJointGroup(segments, clusters[0]) );
			Joints.push_back( setupPointJointGroup(segments, clusters[1]) );
		}
	}

	return Joints;
}



PointJointGroup* JointDetector::setupPointJointGroup( QVector<Primitive*> segments, std::vector<Point>& points )
{
	PointJointGroup *PJG = new PointJointGroup(POINTJOINT);

	// Center of joint
	Point center(0.0, 0.0, 0.0);
	foreach(Point p, points)
		center += p;
	center /= points.size();


	// Set up point-joint
	PJG->pos = center;
	PJG->process(segments);

	return PJG;
}


