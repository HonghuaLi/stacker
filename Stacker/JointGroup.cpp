#include "JointGroup.h"
#include "SimpleDraw.h"
#include "Voxeler.h"


void JointGroup::process( QVector< QString > segments, Vec3d joint )
{
	addNodes(segments);

	// Set the joint position and coordinates in two primitives
	jointPos = joint;

	Primitive * a = getPrimitive(segments.first());
	Primitive * b = getPrimitive(segments.last());
	coordinates[segments.first()] = a->getCoordinate(joint);
	coordinates[segments.last()] = b->getCoordinate(joint);
}

void JointGroup::draw()
{
	// Show joints
	SimpleDraw::IdentifyPoint( jointPos );
}

QVector<Primitive *> JointGroup::regroup()
{
	QVector<Primitive *> result;

	Primitive * frozen = getPrimitive(nodes.values().first());
	Primitive * non_frozen = getPrimitive(nodes.values().last());

	if(frozen->isFrozen == non_frozen->isFrozen || !non_frozen->isAvailable)
		return result;

	// Swap if needed
	if(!frozen->isFrozen) 
	{
		Primitive * temp = frozen;
		frozen = non_frozen;
		non_frozen = temp;
	}


	// The joint has been apart, try to join them again
	Vec3d newPos = frozen->fromCoordinate(coordinates[frozen->id]);
	non_frozen->movePoint(jointPos, newPos - jointPos);

	jointPos = newPos;
	coordinates[frozen->id] = frozen->getCoordinate(jointPos);
	coordinates[non_frozen->id] = non_frozen->getCoordinate(jointPos);


	result.push_back(non_frozen);

	return result;
}
