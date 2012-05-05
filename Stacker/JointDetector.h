#pragma once

#include <vector>

#include <QMap>
#include <QString>

#include "GraphicsLibrary/Mesh/QSurfaceMesh.h"

class Group;
class PointJointGroup;
class Primitive;

class JointDetector
{
public:
	JointDetector();

	QVector<Group*> detect( QVector<Primitive*> primitives );
	QVector<Group*> analyzeIntersection( Primitive* a, Primitive* b, std::vector<Point> &intersection );
	
	PointJointGroup* setupPointJointGroup( QVector<Primitive*> segments, std::vector<Point>& points );
	void twoFurthestPoints(std::vector<Point> &points, Point &p1, Point &p2);
	std::vector< std::vector<Point> > twoClassClustering(std::vector<Point>& points, Point seed1, Point seed2);
	double distanceCluster2Cluster(std::vector<Point> &cluster1, std::vector<Point> &cluster2);


	double JOINT_THRESHOLD;
};