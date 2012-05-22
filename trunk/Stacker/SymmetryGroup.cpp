#include "SymmetryGroup.h"
#include "Utility/SimpleDraw.h"
#include "Primitive.h"
#include "GCylinder.h"

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

	// Reflect the \frozen
	std::vector<Point> pointsB;
	if (frozen->primType == GCYLINDER)
	{
		//GC
		GCylinder * frozen_gc = (GCylinder *) frozen;
		GCylinder * non_frozen_gc = (GCylinder *) non_frozen;

		// Reflect the \basicGC and \curveTranslations
		int N = frozen_gc->gc->crossSection.size();
		pointsB.resize( 2*N );
		for(int i = 0; i < N; i++)
		{
			Point p = frozen_gc->basicGC.crossSection[i].center;
			pointsB[corr[i]] = symmetryPlane.reflection(p);

			// curveTranslate
			Vec3d T = frozen_gc->curveTranslation[i];
			pointsB[N + corr[i]] = symmetryPlane.reflection(T);
		}
	}
	else
	{
		// Cuboid
		std::vector<Point> pointsA = frozen->points();		
		pointsB.resize( pointsA.size() );

		for(int i = 0; i < pointsA.size(); i++)
			pointsB[corr[i]] = symmetryPlane.reflection(pointsA[i]);
	}	

	// Scale
	std::vector<double> scalesA = frozen->scales();
	std::vector<double> scalesB( scalesA.size() );
	for(int i = 0; i < scalesA.size(); i++)
	{
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

Group* SymmetryGroup::clone()
{
	SymmetryGroup * g = new SymmetryGroup(SYMMETRY);

	g->id = this->id;
	g->nodes = this->nodes;
	g->symmetryPlane = this->symmetryPlane;
	g->correspondence = this->correspondence;

	return g;
}
