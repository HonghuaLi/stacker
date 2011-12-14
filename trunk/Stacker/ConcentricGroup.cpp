#include "ConcentricGroup.h"
#include "SimpleDraw.h"

void ConcentricGroup::process( QVector< QString > segments )
{
	addNodes(segments);

	foreach(int i, nodes.keys())
	{
		Primitive * a = getPrimitive(nodes[i]);
		Point cA = a->centerPoint();

		foreach(int j, nodes.keys())
		{
			if(i == j) continue;

			Primitive * b = getPrimitive(nodes[j]);
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
