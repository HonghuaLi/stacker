#include "CoplanarGroup.h"
#include "SimpleDraw.h"

void CoplanarGroup::process( QVector<QString> segments )
{
	addNodes(segments);
	
	Primitive * a = getPrimitive(nodes.values().first());

	std::vector<Vec3d> axis = a->majorAxis();

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

	foreach(int i, nodes.keys())
	{
		Primitive * a = getPrimitive(nodes[i]);

		std::vector< std::vector<Vec3d> > curves = a->getCurves();

		foreach(std::vector<Vec3d> curve, curves)
		{
			Vec3d v = cross(curve[0], curve[1]).normalized();

			int c = 0;

			if(1 - abs(dot(v, groupAxis[0].n)) < EPS)	c = 1;
			if(1 - abs(dot(v, groupAxis[1].n)) < EPS)	c = 2;

			SimpleDraw::IdentifyCurve(curve, color[c][0], color[c][1], color[c][2], 1.0, 20);
		}
	}

	groupAxis[0].draw();
	groupAxis[1].draw();
	groupAxis[2].draw();
}
