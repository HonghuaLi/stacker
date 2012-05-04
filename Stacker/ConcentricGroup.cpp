#include "ConcentricGroup.h"
#include "SimpleDraw.h"
#include "Primitive.h"

void ConcentricGroup::process( QVector< Primitive* > segments )
{
	Group::process(segments);

	Point cA = nodes.first()->centerPoint();
	Point cB = nodes.last()->centerPoint();

	axis = (cA - cB).normalized();
	center = (cA + cB) * 0.5;
}

void ConcentricGroup::regroup()
{
}

void ConcentricGroup::draw()
{
	SimpleDraw::IdentifyArrow(center, center + axis * 1.0);
	SimpleDraw::IdentifyArrow(center, center - axis * 1.0);

	Group::draw();
}
