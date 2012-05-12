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
public:
	bool sliding;
	double JOINT_THRESHOLD;
};