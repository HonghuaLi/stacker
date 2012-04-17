#include "LineJointGroup.h"
#include "Primitive.h"
#include "SimpleDraw.h"


void LineJointGroup::process( QVector< Primitive* > segments )
{
	Group::process(segments);

	// Calculate the joint coordinates in both primitives
	Primitive * a = nodes.first();
	Primitive * b = nodes.last();

	if (lineEnds.isEmpty())
	{
		lineEnds.push_back(Point(0.0, 0.0, 0.0));
		lineEnds.push_back(Point(0.0, 0.0, 0.0));
	}

	lineEndsCoords[a->id].resize(2);
	lineEndsCoords[a->id][0] = a->getCoordinate(lineEnds[0]);
	lineEndsCoords[a->id][1] = a->getCoordinate(lineEnds[1]);

	lineEndsCoords[b->id].resize(2);
	lineEndsCoords[b->id][0] = b->getCoordinate(lineEnds[0]);
	lineEndsCoords[b->id][1] = b->getCoordinate(lineEnds[1]);
}

QVector<QString> LineJointGroup::regroup()
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

	// Regroup
	// ........

	if (non_frozen->isFrozen)
		result.push_back(non_frozen->id);

	return result;
}

void LineJointGroup::draw()
{
	if(nodes.isEmpty()) return;

	// Show joints
	Primitive * a = nodes.first();
	Primitive * b = nodes.last();

	if(isDraw)
	{
		SimpleDraw::IdentifyLine( a->fromCoordinate(lineEndsCoords[a->id][0]),
			a->fromCoordinate(lineEndsCoords[a->id][1]), Vec4d(1, 0, 1, 1) );
		SimpleDraw::IdentifyLine( b->fromCoordinate(lineEndsCoords[b->id][0]), 
			b->fromCoordinate(lineEndsCoords[b->id][0]), Vec4d(0, 1, 0, 1) );
	}

	Group::draw();

}

void LineJointGroup::saveParameters( std::ofstream &outF )
{
	outF << lineEnds[0] << '\t' << lineEnds[1];
}

void LineJointGroup::loadParameters( std::ifstream &inF )
{
	lineEnds.resize(2);
	inF >> lineEnds[0] >> lineEnds[1];
}
