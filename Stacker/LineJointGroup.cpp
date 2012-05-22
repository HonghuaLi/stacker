#include "LineJointGroup.h"
#include "Primitive.h"
#include "Utility/SimpleDraw.h"
#include "Cuboid.h"


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

void LineJointGroup::regroup()
{
	Primitive *frozen,  *non_frozen;
	if (!getRegroupDirection(frozen, non_frozen)) return;

	// Regroup
	Point A = non_frozen->fromCoordinate(lineEndsCoords[non_frozen->id][0]);
	Point B = non_frozen->fromCoordinate(lineEndsCoords[non_frozen->id][1]);
	Point newA = frozen->fromCoordinate(lineEndsCoords[frozen->id][0]);
	Point newB = frozen->fromCoordinate(lineEndsCoords[frozen->id][1]);

	// Line joint is now only for Cuboid
	if (non_frozen->primType == CUBOID)
	{
		Cuboid* cuboid = (Cuboid*) non_frozen;
		cuboid->moveLineJoint( A, B, newA-A, newB-B );
	}

	// Update the line ends
	if (lineEnds.size() != 2) lineEnds.resize(2);
	lineEnds[0] = newA;
	lineEnds[1] = newB;

	// Fix the regrouped line joint
	non_frozen->addFixedPoint(newA);
	non_frozen->addFixedPoint(newB);
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
	updateLineEnds();
	outF << lineEnds[0] << '\t' << lineEnds[1];
}

void LineJointGroup::loadParameters( std::ifstream &inF, Vec3d translation, double scaleFactor )
{
	lineEnds.resize(2);
	inF >> lineEnds[0] >> lineEnds[1];

	lineEnds[0] += translation;
	lineEnds[0] *= scaleFactor;

	lineEnds[1] += translation;
	lineEnds[1] *= scaleFactor;
}

Group* LineJointGroup::clone()
{
	LineJointGroup * g = new LineJointGroup(LINEJOINT);

	g->id = this->id;
	g->nodes = this->nodes;
	g->lineEndsCoords = this->lineEndsCoords;

	return g;
}

void LineJointGroup::updateLineEnds()
{
	Primitive * a = nodes.first();
	lineEnds[0] = a->fromCoordinate(lineEndsCoords[a->id][0]);
	lineEnds[1] = a->fromCoordinate(lineEndsCoords[a->id][1]);
}
