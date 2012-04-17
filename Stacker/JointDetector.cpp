#include "JointDetector.h"

#include "StackerGlobal.h"
#include "Primitive.h"
#include "PointJointGroup.h"
#include "LineJointGroup.h"

#include <algorithm>
#include "GraphicsLibrary/Voxel/Voxeler.h"
#include "MathLibrary/Bounding/MinOBB3.h"

#define POINT_LINE_THRESHOLD 4
#define	LINE_2PONINTS_THRESHOLD 0.3

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

				//// Debug: visualize the intersection
				//foreach (Point p, points)
				//	a->debugPoints.push_back(p);

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

	// Decide the type of joint
	if (box.Extent[2]/box.Extent[1] < POINT_LINE_THRESHOLD)
	{
		// Point-joint
		Joints.push_back( setupPointJointGroup(segments, intersection) );
	}
	else
	{
		// Cluster points into two clusters
		Point p1 = box.Center + box.Extent[2] * box.Axis[2];
		Point p2 = box.Center - box.Extent[2] * box.Axis[2];
		std::vector< std::vector< Point > >clusters = twoClassClustering(intersection, p1, p2);

		// The distance between two clusters
		double dis = distanceCluster2Cluster(clusters[0], clusters[1]);

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

void JointDetector::twoFurthestPoints( std::vector<Point> &points, Point &p1, Point &p2 )
{
	p1 = points[0];
	double maxDis = 0.0;
	foreach( Point p, points)
	{
		double dis = (p-p1).norm();
		if (dis > maxDis){
			p2 = p;
			maxDis = dis;
		}
	}
}


std::vector< std::vector<Point> > JointDetector::twoClassClustering( std::vector<Point>& points, Point seed1, Point seed2 )
{
	// Assign all the points to two seeds
	std::vector<  std::vector< Point > >clusters(2);
	foreach( Vec3d p, points)
	{
		if ((p - seed1).norm() < (p - seed2).norm())
			clusters[0].push_back(p);
		else
			clusters[1].push_back(p);
	}

	return clusters;
}



double JointDetector::distanceCluster2Cluster( std::vector<Point> &cluster1, std::vector<Point> &cluster2 )
{
	double minDis = DOUBLE_INFINITY;
	foreach(Point p1, cluster1)
	{
		foreach(Point p2, cluster2)
		{
			double dis = (p1-p2).norm();

			if (dis < minDis)
			{
				minDis = dis;
			}
		}
	}

	return minDis;
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
	PJG->joint = center;
	PJG->process(segments);

	return PJG;
}


