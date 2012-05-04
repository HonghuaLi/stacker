#include "PointJointGroup.h"

#include "Primitive.h"
#include "Controller.h"
#include "SimpleDraw.h"
#include "GraphicsLibrary/Voxel/Voxeler.h"


PointJointGroup::PointJointGroup( GroupType newType ): Group(newType)
{
	Point p(0., 0., 0.);
	pos = p;
}

void PointJointGroup::process( QVector< Primitive* > segments )
{
	Group::process(segments);

	// Calculate the joint coordinates in both primitives
	Primitive * a = nodes.first();
	Primitive * b = nodes.last();

	jointCoords[a->id] = a->getCoordinate(pos);
	jointCoords[b->id] = b->getCoordinate(pos);
}

void PointJointGroup::regroup()
{
	Primitive *frozen,  *non_frozen;
	if (!getRegroupDirection(frozen, non_frozen)) return;

	// The joint has been apart, try to join them again
	Vec3d newPos = frozen->fromCoordinate(jointCoords[frozen->id]);
	Vec3d oldPos = non_frozen->fromCoordinate(jointCoords[non_frozen->id]);

	non_frozen->movePoint(oldPos, newPos - oldPos);

	// Fix the regrouped point joint
	non_frozen->addFixedPoint(newPos);
}

void PointJointGroup::draw()
{
	if(nodes.isEmpty()) return;

	// Show joints
	Primitive * a = nodes.first();
	Primitive * b = nodes.last();

	if(isDraw)
	{
		SimpleDraw::IdentifyPoint( a->fromCoordinate(jointCoords[a->id]), 1,0,1, 5 );
		SimpleDraw::IdentifyPoint( b->fromCoordinate(jointCoords[b->id]), 0,1,0, 8 );
	}

	Group::draw();
}

void PointJointGroup::saveParameters( std::ofstream &outF )
{
	//Primitive * a = nodes.first();
	//Primitive * b = nodes.last();

	//outF << qPrintable(a->id) << '\t';
	//foreach(double c, jointCoords[a->id])
	//	outF << c << '\t';

	//outF << qPrintable(b->id) << '\t';
	//foreach(double c, jointCoords[b->id])
	//	outF << c << '\t';
	outF << getJointPos();
}

void PointJointGroup::loadParameters( std::ifstream &inF, Vec3d translation, double scaleFactor )
{
	inF >> pos;
	pos += translation;
	pos *= scaleFactor;
}

Point PointJointGroup::getJointPosOnPrimitive( Primitive* prim )
{
	return prim->fromCoordinate(jointCoords[prim->id]);
}

Point PointJointGroup::getJointPos()
{
	Primitive * a = nodes.first();
	return a->fromCoordinate(jointCoords[a->id]);
}




