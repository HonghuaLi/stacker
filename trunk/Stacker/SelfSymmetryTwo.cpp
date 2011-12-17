#include "SelfSymmetryTwo.h"


void SelfSymmetryTwo::process( QVector< QString > segments )
{
	addNodes(segments);

	Primitive * prim = getPrimitive(nodes[0]); 
	selectedPartId = prim->selectedPartId;

	symmetryPlanes = prim->getSymmetryPlanes(2);
}

void SelfSymmetryTwo::draw()
{
	symmetryPlanes[0].draw();
	symmetryPlanes[1].draw();
}

void SelfSymmetryTwo::save( std::ofstream &outF )
{
	Group::save(outF);

	// output the axis
	Primitive * prim = getPrimitive(nodes[0]); 
	prim->selectedPartId = selectedPartId;
	Vec3d fCenter = prim->selectedPartPos();
	Vec3d ceter = prim->centerPoint();
	Vec3d normal = fCenter - ceter;
	normal.normalize();
	outF << normal << "\t";
}

void SelfSymmetryTwo::load( std::ifstream &inF )
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


