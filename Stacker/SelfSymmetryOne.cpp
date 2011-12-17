#include "SelfSymmetryOne.h"


void SelfSymmetryOne::process( QVector< QString > segments )
{
	addNodes(segments);
	Primitive * prim = getPrimitive(nodes[0]); 
	selectedPartId = prim->selectedPartId;

	std::vector<Plane> planes = prim->getSymmetryPlanes(1);
	symmetryPlane = planes[0];
}

void SelfSymmetryOne::draw()
{
	symmetryPlane.draw();
}

void SelfSymmetryOne::save( std::ofstream &outF )
{
	Group::save(outF);

	// output the axis
	Primitive * prim = getPrimitive(nodes[0]); 
	prim->selectedPartId = selectedPartId;
	Vec3d normal = prim->selectedPartPos() - prim->centerPoint();
	normal.normalize();
	outF << normal << "\t";
}

void SelfSymmetryOne::load( std::ifstream &inF )
{
	int n;
	inF >> n;
	std::string str;
	QVector<QString> segments;
	inF >> str;
	segments.push_back(str.c_str());

	Vec3d normal;
	inF >> normal;
	Primitive * prim = getPrimitive(segments[0]); 
	prim->setSelectedPartId(normal);

	process(segments);
}


