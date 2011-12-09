#include "SymmetryGroup.h"

void SymmetryGroup::process(std::vector<int> segments)
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
