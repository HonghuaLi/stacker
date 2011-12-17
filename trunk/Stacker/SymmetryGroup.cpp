#include "SymmetryGroup.h"

void SymmetryGroup::process(QVector<QString> segments)
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

			Point point = (cA + cB) * 0.5;
			Normal normal = (cA - cB).normalized();

			symmetryPlanes.push_back(Plane(normal, point));
		}
	}
}

void SymmetryGroup::draw()
{
	// Center point of each node
	Group::draw();

	foreach(Plane p, symmetryPlanes)
		p.draw();
}
