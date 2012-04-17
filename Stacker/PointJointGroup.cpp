#include "PointJointGroup.h"
#include "SimpleDraw.h"
#include "GraphicsLibrary/Voxel/Voxeler.h"
#include "Controller.h"


PointJointGroup::PointJointGroup( GroupType newType ): Group(newType)
{
	joint = Point(0., 0., 0.);
}

void PointJointGroup::process( QVector< Primitive* > segments )
{
	Group::process(segments);

	// Calculate the joint coordinates in both primitives
	Primitive * a = nodes.first();
	Primitive * b = nodes.last();

	jointCoords[a->id] = a->getCoordinate(joint);
	jointCoords[b->id] = b->getCoordinate(joint);
}

QVector<QString> PointJointGroup::regroup()
{
	QVector<QString> result;

	Primitive * frozen =nodes.first();
	Primitive * non_frozen = nodes.last();

	// Both are frozen or unfrozen
	if(frozen->isFrozen == non_frozen->isFrozen)
		return result;

	// Match the pointer with the correct primitive
	if(!frozen->isFrozen) 
	{
		Primitive * temp = frozen;
		frozen = non_frozen;
		non_frozen = temp;
	}

	// The joint has been apart, try to join them again
	Vec3d newPos = frozen->fromCoordinate(jointCoords[frozen->id]);
	Vec3d oldPos = non_frozen->fromCoordinate(jointCoords[non_frozen->id]);

	Vec3d T = newPos - oldPos;
	non_frozen->movePoint(oldPos, T);

	// Make the joint fixed
	non_frozen->addFixedPoint(newPos);

	if (non_frozen->isFrozen)
		result.push_back(non_frozen->id);

	return result;
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
}

void PointJointGroup::saveParameters( std::ofstream &outF )
{
	outF << true << joint;
}

void PointJointGroup::loadParameters( std::ifstream &inF )
{
	inF >> joint;
}
