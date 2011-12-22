#include "SymmetryGroup.h"
#include "SimpleDraw.h"

#define RADIANS(deg)    ((deg)/180.0 * M_PI)
#define DEGREES(rad)    ((rad)/M_PI * 180.0)

void SymmetryGroup::process(QVector<QString> segments)
{
	addNodes(segments);

	Primitive * a = getPrimitive(segments.first());
	Primitive * b = getPrimitive(segments.last());

	Point cA = a->centerPoint();
	Point cB = b->centerPoint();

	Point point = (cA + cB) * 0.5;
	Normal normal = (cA - cB).normalized();

	symmetryPlane = Plane(normal, point);

	std::vector<Vec3d> pointsA = a->points();
	std::vector<Vec3d> pointsB = b->points();

	correspondence[a->id].resize(pointsA.size());
	correspondence[b->id].resize(pointsB.size());

	// Reflect along symmetry plane
	for(int i = 0; i < pointsA.size(); i++)
	{
		pointsA[i] = symmetryPlane.reflection(pointsA[i]);

		double dist = DBL_MAX;
		int id_closest = -1;

		for(int j = 0; j < pointsB.size(); j++)
		{
			double currDist = (pointsA[i] - pointsB[j]).norm();

			if(currDist < dist)
			{
				id_closest = j;
				dist = currDist;
			}
		}

		correspondence[a->id][i] = id_closest;
		correspondence[b->id][id_closest] = i;
	}
}

void SymmetryGroup::draw()
{
	if(!isDraw) return;

	// Draw debug and stuff
	Group::draw();

	symmetryPlane.draw();

	if(hasSymmetryAxis)
		SimpleDraw::DrawArrowDirected(symmetryAxisPos, symmetryAxis, 2);
}

QVector<Primitive *> SymmetryGroup::regroup()
{
	QVector<Primitive *> result;

	Primitive * frozen = getPrimitive(nodes.values().first());
	Primitive * non_frozen = getPrimitive(nodes.values().last());

	if(frozen->isFrozen == non_frozen->isFrozen || !non_frozen->isAvailable)
		return result;

	// Swap if needed
	if(!frozen->isFrozen) 
	{
		Primitive * temp = frozen;
		frozen = non_frozen;
		non_frozen = temp;
	}

	QVector<int> corr = correspondence[frozen->id];

	// reflect frozen
	std::vector<Vec3d> pointsA = frozen->points();
	std::vector<Vec3d> pointsB = non_frozen->points();

	for(int i = 0; i < pointsA.size(); i++)
	{
		Point reflected = symmetryPlane.reflection(pointsA[i]);

		pointsB[corr[i]] = reflected;
	}
	
	non_frozen->reshapeFromCorners(pointsB);

	result.push_back(non_frozen);

	return result;
}
