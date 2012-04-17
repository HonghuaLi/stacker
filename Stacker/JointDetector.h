#pragma once

#include <vector>

#include <QMap>
#include <QString>

#include "GraphicsLibrary/Voxel/Voxeler.h"
class Group;
class Primitive;

class JointDetector
{
public:

	QVector<Group*> detect( QMap<QString, Primitive*> primitives );
	QVector<Group*> analyzeIntersection( std::vector<Voxel> &intersection );

	//void detectPairwiseJoint();
};