#include "CoplanarGroup.h"
#include "SimpleDraw.h"
#include "Primitive.h"

void CoplanarGroup::process( QVector< Primitive* > segments )
{
	Group::process(segments);

	std::vector<Vec3d> axis = nodes.first()->majorAxis();

	groupAxis[0] = Plane(axis[0]);
	groupAxis[1] = Plane(axis[1]);
	groupAxis[2] = Plane(axis[2]);
}

void CoplanarGroup::draw()
{
	Color color[3];
	color[0] = Color(1,0,0,1);
	color[1] = Color(0,1,0,1);
	color[2] = Color(0,0,1,1);

	double EPS = 0.5;

	foreach(Primitive * a, nodes)
	{
		std::vector< std::vector<Vec3d> > curves = a->getCurves();

		foreach(std::vector<Vec3d> curve, curves)
		{
			// find curve center
			Vec3d curveCenter(0,0,0);
			foreach(Point p, curve)	curveCenter += p;
			curveCenter /= curve.size();

			Vec3d curveAxis = cross(curve[0] - curveCenter, curve[1] - curveCenter).normalized();

			int c = 0;

			double d1 = abs(dot(curveAxis, groupAxis[0].n));
			double d2 = abs(dot(curveAxis, groupAxis[1].n));
			double d3 = abs(dot(curveAxis, groupAxis[2].n));

			if(d1 > d2) c = 1;
			if(d2 > d3) c = 2;

			for(uint k = 0; k < curve.size(); k++)
			{
				Vec3d delta = curveAxis * 0.01;

				curve[k] += delta;
			}

			// Draw as closed loop
			curve.push_back(curve.front());
			SimpleDraw::IdentifyCurve(curve, color[c][0], color[c][1], color[c][2], 1.0, 15);
		}
	}

	// Debug:
	//groupAxis[0].draw();
	//groupAxis[1].draw();
	//groupAxis[2].draw();
}

void CoplanarGroup::regroup()
{

	Group::regroup();
}
