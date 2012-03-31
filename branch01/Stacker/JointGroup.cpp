#include "JointGroup.h"
#include "SimpleDraw.h"
#include "Voxeler.h"
#include "Controller.h"


void JointGroup::process( QVector< QString > segments, Vec3d joint )
{
	nodes.clear();

	addNodes(segments);

	// Set the joint position and coordinates in two primitives
	Primitive * a = getPrimitive(segments.first());
	Primitive * b = getPrimitive(segments.last());
	coordinates[segments.first()] = a->getCoordinate(joint);
	coordinates[segments.last()] = b->getCoordinate(joint);
}

void JointGroup::process( QVector< QString > segments )
{
	process(segments, Point(0,0,0));
}

void JointGroup::draw()
{
	if(nodes.isEmpty()) return;

	// Show joints
	Primitive * a = getPrimitive(nodes.values().first());
	Primitive * b = getPrimitive(nodes.values().last());

	if(isDraw)
	{
		SimpleDraw::IdentifyPoint( a->fromCoordinate(coordinates[a->id]), 1,0,1, 5 );
		SimpleDraw::IdentifyPoint( b->fromCoordinate(coordinates[b->id]), 0,1,0, 8 );
	}
}

QVector<QString> JointGroup::regroup()
{
	QVector<QString> result;

	Primitive * frozen = getPrimitive(nodes.values().first());
	Primitive * non_frozen = getPrimitive(nodes.values().last());

	if(frozen->isFrozen == non_frozen->isFrozen)
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
	Vec3d oldPos = non_frozen->fromCoordinate(coordinates[non_frozen->id]);

	Vec3d T = newPos - oldPos;
	non_frozen->movePoint(oldPos, T);

	// Make the joint fixed
	non_frozen->addFixedPoint(newPos);

	if (non_frozen->isFrozen)
		result.push_back(non_frozen->id);

	return result;
}

void JointGroup::save( std::ofstream &outF )
{
	Group::save(outF);

	outF << getJoint();
}

void JointGroup::load( std::ifstream &inF )
{
	int n;
	inF >> n;
	std::string str;
	QVector<QString> segments;
	for (int i=0;i<n;i++)
	{
		inF >> str;
		segments.push_back(str.c_str());
	}

	Vec3d joint;
	inF >> joint;

	process(segments, joint);

}

Point JointGroup::getJoint()
{
	Primitive * primA = getPrimitive(nodes.values().first());
	Primitive * primB = getPrimitive(nodes.values().last());

	Vec3d posA = primA->fromCoordinate(coordinates[primA->id]);
	Vec3d posB = primB->fromCoordinate(coordinates[primB->id]);

	return (posA + posB) / 2;
}
