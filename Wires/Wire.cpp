#include "Wire.h"

#include "SimpleDraw.h"
#include <QtOpenGL>

Wire::Wire()
{
	
}

void Wire::addPoint( const Point& p, uint pindex )
{
	points[pindex] = p;
}

void Wire::addEdge( const Point& p1, const Point& p2, uint pi1, uint pi2, double weight )
{
	points[pi1] = p1;
	points[pi2] = p2;

	edges.insert(SimpleEdge(pi1, pi2, weight));
}

void Wire::draw()
{
	glDisable(GL_LIGHTING);
	glLineWidth(6);
	glColor3f(0,0,1);

	glBegin(GL_LINES);

	for(SimpleEdgeSet::iterator e = edges.begin(); e != edges.end(); e++)
	{
		SimpleEdge edge = *e;

		glVertex3dv(points[edge(0)]);
		glVertex3dv(points[edge(1)]);
	}

	glEnd();

	glEnable(GL_LIGHTING);
}