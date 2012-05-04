#include "SymmetryGroup.h"
#include "SimpleDraw.h"
#include "Primitive.h"

void SymmetryGroup::process(QVector<Primitive*> segments)
{
	// Store the nodes
	Group::process(segments);

	// Compute the symmetry plane
	Primitive * a = segments.first();
	Primitive * b = segments.last();

	Point cA = a->centerPoint();
	Point cB = b->centerPoint();

	Point point = (cA + cB) * 0.5;
	Normal normal = (cA - cB).normalized();
	symmetryPlane = Plane(normal, point);

	// Representative points of two primitives
	std::vector<Vec3d> pointsA = a->points();
	std::vector<Vec3d> pointsB = b->points();

	// Correspondences from both directions
	correspondence[a->id].resize(pointsA.size());
	correspondence[b->id].resize(pointsB.size());

	for(int i = 0; i < pointsA.size(); i++)
	{
		pointsA[i] = symmetryPlane.reflection(pointsA[i]);

		double dist = DBL_MAX;
		int id_closest = -1;

		// Find the point on \b closest to the reflected point on \a
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


void SymmetryGroup::regroup()
{
	Primitive *frozen,  *non_frozen;
	if (!getRegroupDirection(frozen, non_frozen)) return;

	// The correspondence from frozen to unfrozen
	QVector<int> corr = correspondence[frozen->id];

	// Reflect the representative points of \frozen
	std::vector<Point> pointsA = frozen->points();
	std::vector<double> scalesA = frozen->scales();
	int N = pointsA.size();
	std::vector<Point> pointsB( N );
	std::vector<double> scalesB( N );

	for(int i = 0; i < pointsA.size(); i++)
	{
		Point reflected = symmetryPlane.reflection(pointsA[i]);
		pointsB[corr[i]] = reflected;
		scalesB[corr[i]] = scalesA[i];
	}
	
	// Reconstruct the primitive
	non_frozen->reshape(pointsB, scalesB);
}


void SymmetryGroup::draw()
{
	if(!isDraw) return;

	glColor4d(1,1,1,0.5);
	symmetryPlane.draw(0.1);

	// Draw debug and stuff
	Group::draw();
}