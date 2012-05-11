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

private:
	PointJointGroup* setupPointJointGroup( QVector<Primitive*> segments, std::vector<Point>& points );
	QVector<Group*> analyzeIntersection( Primitive* a, Primitive* b, std::vector<Point> &intersection );
	void twoFurthestPoints(std::vector<Point> &points, Point &p1, Point &p2);
	std::vector< std::vector<Point> > twoClassClustering(std::vector<Point>& points, Point seed1, Point seed2);
	double distanceCluster2Cluster(std::vector<Point> &cluster1, std::vector<Point> &cluster2);

public:
	bool sliding;
	double JOINT_THRESHOLD;
};