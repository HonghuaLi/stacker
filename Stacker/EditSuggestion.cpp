#include "EditSuggestion.h"
#include "SimpleDraw.h"
#include "ColorMap.h"

EditSuggestion::EditSuggestion( Point Center, Vec3d Direction, double Value /*= 0.0*/ )
{
	this->center = center;
	this->direction = Direction;
	this->value = Value;
}

EditSuggestion::EditSuggestion()
{
	this->center = Vec3d(0,0,0);
	this->direction = Vec3d(1,0,0);
	this->value = 0;
}

void EditSuggestion::draw(double scale, Vec3d normal)
{
	//std::cout << "center = " << this->center  << "direction = " << this->direction << std::endl;

	std::vector<Point> arrow, tail;
	Vec3d pos = center + direction;

	Vec3d v = direction.normalized() * scale * value;
	Vec3d u = cross(v, normal).normalized() * scale * value;

	arrow.push_back(pos);
	arrow.push_back(pos + u - v);
	arrow.push_back(pos + v);
	arrow.push_back(pos - u - v);

	tail.push_back(center);
	tail.push_back(pos + 0.5*u - 0.5*v);
	tail.push_back(pos);
	tail.push_back(pos - 0.5*u - 0.5*v);

	glDisable(GL_LIGHTING);
	glClear(GL_DEPTH);

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

	SimpleDraw::DrawSphere(center, 0.02f);

	// Outline
	glColor3d(0,0,0);
	glBegin(GL_LINE_STRIP);
	foreach(Point p, arrow) glVertex3dv(p);
	glEnd();


	glEnable(GL_LIGHTING);
}
