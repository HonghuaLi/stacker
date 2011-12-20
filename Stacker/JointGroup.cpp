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
	coordinates.push_back(a->getCoordinate(joint));
	coordinates.push_back(b->getCoordinate(joint));
}

void JointGroup::draw()
{
	// Show joints
	SimpleDraw::IdentifyPoint( jointPos );
}
