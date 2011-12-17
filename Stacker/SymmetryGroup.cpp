#include "SymmetryGroup.h"
#include "SimpleDraw.h"

#define RADIANS(deg)    ((deg)/180.0 * M_PI)
#define DEGREES(rad)    ((rad)/M_PI * 180.0)

void SymmetryGroup::process(QVector<QString> segments)
{
	addNodes(segments);

	std::vector<Plane> potentialSP, redundant;

	// Find potential symmetry planes
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

			potentialSP.push_back(Plane(normal, point));
		}

		allCenters.push_back(cA);
	}

	int expectedNum = segments.size() - 1;
	QVector<bool> visited (potentialSP.size(), false);

	// Filter out redundancy
	for(uint i = 0; i < potentialSP.size(); i++)
	{
		Plane a = potentialSP[i];

		QSet <uint> symmetrySet;

		for(uint j = 0; j < potentialSP.size(); j++)
		{
			if(i == j) continue;

			Plane b = potentialSP[j];

			if(dot(a.n, b.n) < 0) a.n *= -1;

			double theta = DEGREES(acos(RANGED(-1, dot(a.n, b.n), 1)));

			if(theta < 1) symmetrySet.insert(j);
		}

		if(symmetrySet.size() == expectedNum)
		{
			if(!visited[i])	symmetryPlanes.push_back(a);
			
			visited[i] = true;

			foreach(uint p, symmetrySet) visited[p] = true;
		}
	}

	hasSymmetryAxis = false;
	
	if(segments.size() > 4)
	{
		hasSymmetryAxis = true;

		// Position on axis
		Vec3d avg(0,0,0); foreach(Point p, allCenters) avg += p;
		avg /= segments.size();

		symmetryAxisPos = avg;

		symmetryAxis = cross( allCenters[0] - avg, allCenters[1] - avg );
	}
}

void SymmetryGroup::draw()
{
	// Center point of each node
	Group::draw();

	foreach(Plane p, symmetryPlanes)
		p.draw();

	if(hasSymmetryAxis)
		SimpleDraw::DrawArrowDirected(symmetryAxisPos, symmetryAxis, 2);
}
