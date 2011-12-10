#include "ConcentricGroup.h"
#include "SimpleDraw.h"

void ConcentricGroup::process( std::vector<int> segments )
{
	addNodes(segments);

	foreach(int i, nodes)
	{
		Primitive * a = getPrimitive(i);
		Point cA = a->centerPoint();

		foreach(int j, nodes)
		{
			if(i == j) continue;

			Primitive * b = getPrimitive(j);
			Point cB = b->centerPoint();

			axis = (cA - cB).normalized();
			center = (cA + cB) * 0.5;
			break;
		}
		break;
	}
}

void ConcentricGroup::draw()
{
	SimpleDraw::IdentifyArrow(center, center + axis * 1.0);
	SimpleDraw::IdentifyArrow(center, center - axis * 1.0);
}
