#include "EditPath.h"
#include "SimpleDraw.h"
#include "ColorMap.h"

EditPath::EditPath( Point Center, Vec3d Direction, double Value /*= 0.0*/ )
{
	this->center = center;
	this->move = Direction;
	this->value = Value;
}

EditPath::EditPath()
{
	this->center = Vec3d(0,0,0);
	this->move = Vec3d(1,0,0);
	this->value = 0;
}

void EditPath::draw()
{
	double scale = 0.05;

	Vec3d pos = center + move;

	double value3 = value * value * value;
	if (value3 < 0.1)
	{
		SimpleDraw::IdentifyLine(center, pos, Color(0.1, 0.1, 0.6, 0.2), false, 1.0);
		return;
	}

	std::vector<Point> arrow, tail;

	Vec3d normal = move[2]? Vec3d(1,0,0) : Vec3d(0, 0, 1);
	Vec3d v = move.normalized() * scale * value3;
	Vec3d u = cross(v, normal).normalized() * scale * value3;

	arrow.push_back(pos);
	arrow.push_back(pos + u - v);
	arrow.push_back(pos + v);
	arrow.push_back(pos - u - v);

	tail.push_back(center);
	tail.push_back(pos + 0.5*u - 0.5*v);
	tail.push_back(pos);
	tail.push_back(pos - 0.5*u - 0.5*v);

	glDisable(GL_LIGHTING);
	glClear(GL_DEPTH_BUFFER_BIT);

	// Color
	uchar * rgb = new uchar[3];	
	ColorMap::jetColorMap(rgb, value, 0, 1);
	glColor3ubv(rgb);
	glLineWidth(2);

	// arrow
	glBegin(GL_QUADS);
	foreach(Point p, arrow) glVertex3dv(p);
	glEnd();

	// tail
	glColor4ub(rgb[0], rgb[1], rgb[2], 100);
	glEnable(GL_BLEND); 
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUADS);
	foreach(Point p, tail) glVertex3dv(p);
	glEnd();

	// Center
	SimpleDraw::DrawSphere(center, 0.02f);

	// Outline
	glColor3d(0.2,0.2,0.2);
	glBegin(GL_LINE_STRIP);
	foreach(Point p, arrow) glVertex3dv(p);
	glEnd();


	glEnable(GL_LIGHTING);
}
